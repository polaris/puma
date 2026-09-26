cmake_minimum_required(VERSION 3.21)

project(puma VERSION 1.0 LANGUAGES CXX)

if(WIN32)
    # Target Windows 10. NTDDI_VERSION must be set explicitly: the SDK derives only
    # NTDDI_WIN10 from _WIN32_WINNT, and the socket timestamping API
    # (SIO_TIMESTAMPING, SO_TIMESTAMP) is gated behind NTDDI_WIN10_FE.
    add_compile_definitions(_WIN32_WINNT=0x0A00 NTDDI_VERSION=0x0A00000A)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(PUMA_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(PUMA_NATIVE_ARCH "Optimize release builds for the host CPU (binaries are not portable)" OFF)
option(PUMA_SANITIZE_THREAD "Build with ThreadSanitizer (development only, needs its own build dir)" OFF)

if(MSVC)
    # C4324: structure padded due to alignment specifier. That padding is the point
    # of the alignas() cache-line separation in spsc_ring.h and packet_ring.h.
    add_compile_options(/W4 /permissive- /wd4324 $<$<BOOL:${PUMA_WARNINGS_AS_ERRORS}>:/WX>)
else()
    add_compile_options(
        -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Wold-style-cast
        -Wcast-align -Woverloaded-virtual -Wnull-dereference -Wdouble-promotion
        -Wimplicit-fallthrough
        $<$<BOOL:${PUMA_WARNINGS_AS_ERRORS}>:-Werror>)
endif()

if(MSVC)
    add_compile_options($<$<CONFIG:Release>:/Oi> $<$<CONFIG:Release>:/Gy>)
    add_link_options($<$<CONFIG:Release>:/OPT:REF> $<$<CONFIG:Release>:/OPT:ICF>)
elseif(PUMA_NATIVE_ARCH)
    add_compile_options($<$<CONFIG:Release>:-march=native>)
endif()

if(PUMA_SANITIZE_THREAD)
    if(MSVC)
        message(FATAL_ERROR "PUMA_SANITIZE_THREAD is not supported with MSVC")
    endif()
    add_compile_options(-fsanitize=thread -fno-omit-frame-pointer -g)
    add_link_options(-fsanitize=thread)
endif()

include(CheckIPOSupported)
check_ipo_supported(RESULT PUMA_IPO_SUPPORTED OUTPUT PUMA_IPO_OUTPUT LANGUAGES CXX)
if(PUMA_IPO_SUPPORTED AND NOT PUMA_SANITIZE_THREAD)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
elseif(PUMA_IPO_SUPPORTED)
    message(STATUS "IPO/LTO disabled because PUMA_SANITIZE_THREAD is on")
else()
    message(STATUS "IPO/LTO not supported: ${PUMA_IPO_OUTPUT}")
endif()

find_package(asio REQUIRED)
find_package(miniaudio REQUIRED)
find_package(CLI11 REQUIRED)
find_package(Catch2 3 REQUIRED)

add_subdirectory(netint)
add_subdirectory(netint-demo)
add_subdirectory(dll)
add_subdirectory(sender)
add_subdirectory(receiver)
add_subdirectory(clocksync)
add_subdirectory(clocksync-demo)
add_subdirectory(streaming)

enable_testing()
add_subdirectory(tests)
