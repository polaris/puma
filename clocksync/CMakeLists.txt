add_library(clocksync STATIC clock_sync.cpp sync_message.cpp)
target_include_directories(clocksync PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(clocksync PRIVATE asio::asio)
target_compile_features(clocksync PUBLIC cxx_std_20)