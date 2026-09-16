cmake_minimum_required(VERSION 3.21)

project(puma VERSION 1.0 LANGUAGES CXX)

if(WIN32)
    # Target Windows 10. NTDDI_VERSION must be set explicitly: the SDK derives only
    # NTDDI_WIN10 from _WIN32_WINNT, and the socket timestamping API
    # (SIO_TIMESTAMPING, SO_TIMESTAMP) is gated behind NTDDI_WIN10_FE.
    add_compile_definitions(_WIN32_WINNT=0x0A00 NTDDI_VERSION=0x0A00000A)
endif()

find_package(asio REQUIRED)
find_package(miniaudio REQUIRED)
find_package(CLI11 REQUIRED)

add_subdirectory(netint)
add_subdirectory(netint-demo)
add_subdirectory(dll)
add_subdirectory(sender)
add_subdirectory(receiver)
add_subdirectory(clocksync)
add_subdirectory(clocksync-demo)
