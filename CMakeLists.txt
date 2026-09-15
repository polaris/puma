cmake_minimum_required(VERSION 3.21)

project(puma VERSION 1.0 LANGUAGES CXX)

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
