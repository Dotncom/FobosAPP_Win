cmake_minimum_required(VERSION 3.22.1)

project(fobos_native LANGUAGES CXX)

add_library(fobos_native SHARED
        fobos_native.cpp)

target_compile_features(fobos_native PRIVATE cxx_std_17)

target_link_libraries(fobos_native
        log)
