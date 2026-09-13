add_executable(netint-demo demo.cpp)
target_include_directories(netint-demo PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_compile_features(netint-demo PRIVATE cxx_std_20)
target_link_libraries(netint-demo PRIVATE netint asio::asio)
