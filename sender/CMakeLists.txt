add_executable(sender sender.cpp)
target_compile_features(sender PRIVATE cxx_std_20)
target_link_libraries(sender PRIVATE netint streaming clocksync miniaudio::miniaudio asio::asio CLI11::CLI11)