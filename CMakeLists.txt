cmake_minimum_required(VERSION 3.16)
project(free-space-pibt CXX)

set(CMAKE_OSX_SYSROOT "/Library/Developer/CommandLineTools/SDKs/MacOSX12.3.sdk")
set(CMAKE_CXX_STANDARD             20)

add_subdirectory(./free-space-pibt)

add_executable(free-space-mapf free-space-mapf.cpp)
target_compile_features(free-space-mapf PUBLIC cxx_std_17)
target_link_libraries(free-space-mapf lib-mapf)

# format
add_custom_target(clang-format
  COMMAND clang-format -i
  ../free-space-pibt/include/*.hpp
  ../free-space-pibt/src/*.cpp
  ../free-space-mapf.cpp)