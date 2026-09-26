add_executable(receiver receiver.cpp status_reporter.cpp)
target_compile_features(receiver PRIVATE cxx_std_20)
target_include_directories(receiver PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(receiver PRIVATE netint streaming miniaudio::miniaudio asio::asio CLI11::CLI11)    