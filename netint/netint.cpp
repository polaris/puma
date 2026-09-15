#include "netint.h"

#include <algorithm>
#include <cstring>
#include <tuple>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>
#include <windows.h>
#pragma comment(lib, "Iphlpapi.lib")
#else
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>
#endif

namespace net {
namespace {

/// sin_addr.s_addr is already in network byte order, which is the byte order
/// address_v4::bytes_type expects. Copying avoids ntohl(), which is a macro on
/// some platforms (macOS) and so cannot be written as ::ntohl.
asio::ip::address_v4 toAddressV4(const in_addr& addr) {
    asio::ip::address_v4::bytes_type bytes{};
    std::memcpy(bytes.data(), &addr.s_addr, bytes.size());
    return asio::ip::address_v4(bytes);
}

#if defined(_WIN32)

std::string narrow(const wchar_t* wide) {
    if (!wide) return {};
    const int n = ::WideCharToMultiByte(CP_UTF8, 0, wide, -1, nullptr, 0, nullptr, nullptr);
    if (n <= 1) return {};
    std::string out(static_cast<std::size_t>(n - 1), '\0');
    ::WideCharToMultiByte(CP_UTF8, 0, wide, -1, out.data(), n, nullptr, nullptr);
    return out;
}

std::vector<Interface> enumerateNative() {
    std::vector<Interface> out;

    const ULONG flags = GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST |
                        GAA_FLAG_SKIP_DNS_SERVER;

    ULONG size = 15000;  // the documented starting point
    std::vector<unsigned char> buffer(size);
    ULONG rc = 0;

    for (int attempt = 0; attempt < 3; ++attempt) {
        auto* adapters = reinterpret_cast<IP_ADAPTER_ADDRESSES*>(buffer.data());
        rc = ::GetAdaptersAddresses(AF_INET, flags, nullptr, adapters, &size);
        if (rc != ERROR_BUFFER_OVERFLOW) break;
        buffer.assign(size, 0);
    }
    if (rc != NO_ERROR) return out;

    const auto* adapters = reinterpret_cast<const IP_ADAPTER_ADDRESSES*>(buffer.data());
    for (const auto* a = adapters; a; a = a->Next) {
        if (a->OperStatus != IfOperStatusUp) continue;
        if (a->IfType == IF_TYPE_SOFTWARE_LOOPBACK) continue;
        if (a->Flags & IP_ADAPTER_NO_MULTICAST) continue;

        for (const auto* u = a->FirstUnicastAddress; u; u = u->Next) {
            const sockaddr* sa = u->Address.lpSockaddr;
            if (!sa || sa->sa_family != AF_INET) continue;
            const auto* sin = reinterpret_cast<const sockaddr_in*>(sa);

            Interface iface;
            iface.name        = a->AdapterName ? a->AdapterName : "";
            iface.description = narrow(a->FriendlyName);
            iface.address     = toAddressV4(sin->sin_addr);
            iface.index       = a->IfIndex;
            out.push_back(std::move(iface));
        }
    }
    return out;
}

#else

std::vector<Interface> enumerateNative() {
    std::vector<Interface> out;

    ifaddrs* list = nullptr;
    if (::getifaddrs(&list) != 0) return out;

    for (const ifaddrs* p = list; p; p = p->ifa_next) {
        if (!p->ifa_addr || p->ifa_addr->sa_family != AF_INET) continue;

        const unsigned flags = p->ifa_flags;
        if (!(flags & IFF_UP)) continue;
        if (!(flags & IFF_MULTICAST)) continue;
        if (flags & IFF_LOOPBACK) continue;

        const auto* sin = reinterpret_cast<const sockaddr_in*>(p->ifa_addr);

        Interface iface;
        iface.name    = p->ifa_name ? p->ifa_name : "";
        iface.address = toAddressV4(sin->sin_addr);
        iface.index   = iface.name.empty() ? 0u : ::if_nametoindex(iface.name.c_str());
        out.push_back(std::move(iface));
    }

    ::freeifaddrs(list);
    return out;
}

#endif

}  // namespace

std::vector<Interface> enumerate() {
    auto out = enumerateNative();
    std::sort(out.begin(), out.end(), [](const Interface& a, const Interface& b) {
        return std::tie(a.name, a.address) < std::tie(b.name, b.address);
    });
    return out;
}

std::optional<Interface> find(std::string_view wanted) {
    if (wanted.empty()) return std::nullopt;
    for (auto& iface : enumerate()) {
        if (iface.name == wanted) return iface;
        if (!iface.description.empty() && iface.description == wanted) return iface;
        if (iface.address.to_string() == wanted) return iface;
    }
    return std::nullopt;
}

std::optional<Interface> selectDefault() {
    auto all = enumerate();
    if (all.size() == 1) return all.front();
    return std::nullopt;
}

void configureSender(asio::ip::udp::socket& socket,
                     const asio::ip::udp::endpoint& group,
                     const Interface& iface,
                     const MulticastOptions& options) {
    socket.open(asio::ip::udp::v4());
    socket.set_option(asio::ip::multicast::outbound_interface(iface.address));
    socket.set_option(asio::ip::multicast::hops(options.hops));
    socket.set_option(asio::ip::multicast::enable_loopback(options.loopback));
    socket.connect(group);
}

void configureReceiver(asio::ip::udp::socket& socket,
                       const asio::ip::udp::endpoint& group,
                       const Interface& iface) {
    socket.open(asio::ip::udp::v4());
    socket.set_option(asio::ip::udp::socket::reuse_address(true));
    socket.bind(asio::ip::udp::endpoint(asio::ip::address_v4::any(), group.port()));
    socket.set_option(
        asio::ip::multicast::join_group(group.address().to_v4(), iface.address));
}

void configureBidirectional(asio::ip::udp::socket& socket,
                            const asio::ip::udp::endpoint& group,
                            const Interface& iface,
                            const MulticastOptions& options) {
    configureReceiver(socket, group, iface);
    socket.set_option(asio::ip::multicast::outbound_interface(iface.address));
    socket.set_option(asio::ip::multicast::hops(options.hops));
    socket.set_option(asio::ip::multicast::enable_loopback(options.loopback));
}

}  // namespace net