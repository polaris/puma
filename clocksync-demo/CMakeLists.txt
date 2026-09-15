add_executable(clocksync-demo demo.cpp)
target_include_directories(clocksync-demo PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_compile_features(clocksync-demo PRIVATE cxx_std_20)
target_link_libraries(clocksync-demo PRIVATE clocksync netint asio::asio CLI11::CLI11)
