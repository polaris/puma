add_executable(sender sender.cpp)
target_compile_features(sender PRIVATE cxx_std_20)
target_link_libraries(sender PRIVATE netint miniaudio::miniaudio asio::asio)