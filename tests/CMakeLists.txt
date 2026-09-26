add_executable(puma_tests
    test_byte_order.cpp
    test_sync_message.cpp
    test_servo.cpp
    test_spsc_ring.cpp
    test_packet_ring.cpp
    test_time_filter.cpp
    test_servo_additions.cpp
    test_frame_ring.cpp
    test_audio_packet.cpp
)

target_compile_features(puma_tests PRIVATE cxx_std_20)
target_include_directories(puma_tests PRIVATE ${PROJECT_SOURCE_DIR}/include)

target_compile_definitions(puma_tests PRIVATE FRAME_RING_CHECKS=1)

target_link_libraries(puma_tests PRIVATE clocksync streaming miniaudio::miniaudio Catch2::Catch2WithMain)

include(Catch OPTIONAL RESULT_VARIABLE PUMA_CATCH_MODULE)
if(PUMA_CATCH_MODULE)
    catch_discover_tests(puma_tests)
else()
    add_test(NAME puma_tests COMMAND puma_tests)
endif()
