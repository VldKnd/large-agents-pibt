cmake_minimum_required(VERSION 3.16)
project(large-agents-pibt CXX)

set(CMAKE_OSX_SYSROOT "/Library/Developer/CommandLineTools/SDKs/MacOSX12.3.sdk")
set(CMAKE_CXX_STANDARD             20)

set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(./large-agents-pibt)

add_executable(large-agents-mapf large-agents-mapf.cpp)
target_compile_features(large-agents-mapf PUBLIC cxx_std_17)
target_link_libraries(large-agents-mapf lib-mapf)