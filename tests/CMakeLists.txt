add_executable(vergil_tests
    test_byte_order.cpp
    test_sync_message.cpp
    test_servo.cpp
    test_spsc_ring.cpp
    test_packet_ring.cpp
    test_time_filter.cpp
    test_servo_additions.cpp
    test_frame_ring.cpp
    test_audio_packet.cpp
    test_resampler.cpp
    test_rate_loop.cpp
    test_presentation_timeline.cpp
)

target_compile_features(vergil_tests PRIVATE cxx_std_20)
target_include_directories(vergil_tests PRIVATE ${PROJECT_SOURCE_DIR}/include)

target_compile_definitions(vergil_tests PRIVATE FRAME_RING_CHECKS=1)

target_link_libraries(vergil_tests PRIVATE clocksync streaming miniaudio::miniaudio Catch2::Catch2WithMain)

include(Catch OPTIONAL RESULT_VARIABLE VERGIL_CATCH_MODULE)
if(VERGIL_CATCH_MODULE)
    catch_discover_tests(vergil_tests)
else()
    add_test(NAME vergil_tests COMMAND vergil_tests)
endif()
