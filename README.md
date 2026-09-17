# 1. Install dependencies and generate the toolchain and presets
conan install . --output-folder=build --build=missing -s build_type=Release

# 2. Configure
cmake --preset conan-release -DPUMA_NATIVE_ARCH=ON

# 3. Build
cmake --build --preset conan-release
