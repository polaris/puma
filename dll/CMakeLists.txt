add_executable(dll dll.cpp)
target_include_directories(dll PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_compile_features(dll PRIVATE cxx_std_20)
target_link_libraries(dll PRIVATE miniaudio::miniaudio)
