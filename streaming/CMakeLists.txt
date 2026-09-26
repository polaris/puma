add_library(streaming STATIC audio_packet.cpp)
target_include_directories(streaming PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_features(streaming PUBLIC cxx_std_20)