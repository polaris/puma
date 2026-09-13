#ifndef NET_NETINT_H
#define NET_NETINT_H

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <asio.hpp>

namespace net {

/// One IPv4 address on one multicast-capable interface.
/// An interface with several IPv4 addresses yields several entries.
struct Interface {
    std::string name;         ///< "en0", "eth0"; on Windows the adapter GUID
    std::string description;  ///< Windows FriendlyName ("Wi-Fi"); empty on POSIX
    asio::ip::address_v4 address;
    unsigned index = 0;       ///< if_nametoindex() / IfIndex; needed for IPv6
};

/// Interfaces that are up, multicast-capable and not loopback.
/// Sorted by name then address so the order is reproducible.
/// Returns an empty vector if enumeration fails.
[[nodiscard]] std::vector<Interface> enumerate();

/// Match by name, by description, or by dotted-quad address.
[[nodiscard]] std::optional<Interface> find(std::string_view wanted);

/// The single candidate, or nullopt if there are none or more than one.
/// Ambiguity is reported rather than guessed: picking the "first" interface
/// on a host with both Wi-Fi and Ethernet up is exactly the mistake that
/// makes multicast silently go nowhere.
[[nodiscard]] std::optional<Interface> selectDefault();

struct MulticastOptions {
    int hops = 1;          ///< TTL; 1 confines traffic to the local segment
    bool loopback = true;  ///< deliver our own datagrams back to this host
};

/// Open, pin to `iface`, and connect to `group` so send() takes no endpoint.
void configureSender(asio::ip::udp::socket& socket,
                     const asio::ip::udp::endpoint& group,
                     const Interface& iface,
                     const MulticastOptions& options = {});

/// Open, bind to the group's port, and join the group on `iface`.
void configureReceiver(asio::ip::udp::socket& socket,
                       const asio::ip::udp::endpoint& group,
                       const Interface& iface);

}  // namespace net

#endif  // NET_NETINT_H