add_executable(puma_tests
    test_byte_order.cpp
    test_sync_message.cpp
    test_servo.cpp
    test_spsc_ring.cpp
    test_packet_ring.cpp
    test_time_filter.cpp
)

# The Conan toolchain pins CMAKE_CXX_STANDARD to 14; the code under test needs 20.
target_compile_features(puma_tests PRIVATE cxx_std_20)
target_include_directories(puma_tests PRIVATE ${PROJECT_SOURCE_DIR}/include)

# clocksync supplies servo.cpp and sync_message.cpp; miniaudio is only for the
# ma_uint32 that time_filter.h pulls in. Catch2WithMain provides main().
target_link_libraries(puma_tests PRIVATE clocksync miniaudio::miniaudio Catch2::Catch2WithMain)

# catch_discover_tests() lives in Catch2's Catch.cmake, which Conan ships in the
# package and puts on CMAKE_MODULE_PATH via the toolchain, but does not register
# as a package build module. Fall back to a single CTest entry without it.
include(Catch OPTIONAL RESULT_VARIABLE PUMA_CATCH_MODULE)
if(PUMA_CATCH_MODULE)
    catch_discover_tests(puma_tests)
else()
    add_test(NAME puma_tests COMMAND puma_tests)
endif()
