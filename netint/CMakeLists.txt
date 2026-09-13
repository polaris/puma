add_library(netint STATIC netint.cpp)

target_include_directories(netint PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(netint PRIVATE asio::asio)
target_compile_features(netint PRIVATE cxx_std_20)
