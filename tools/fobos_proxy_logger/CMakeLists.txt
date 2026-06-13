cmake_minimum_required(VERSION 3.16)

project(fobos_proxy_logger LANGUAGES CXX)

add_library(fobos_proxy_logger SHARED fobos_proxy_logger.cpp)
target_compile_features(fobos_proxy_logger PRIVATE cxx_std_17)
set_target_properties(fobos_proxy_logger PROPERTIES
    OUTPUT_NAME fobos
    PREFIX ""
)

add_library(fobos_sdr_proxy_logger SHARED fobos_sdr_proxy_logger.cpp)
target_compile_features(fobos_sdr_proxy_logger PRIVATE cxx_std_17)
set_target_properties(fobos_sdr_proxy_logger PROPERTIES
    OUTPUT_NAME fobos_sdr
    PREFIX ""
)

if(MSVC)
    target_compile_options(fobos_proxy_logger PRIVATE /W4 /permissive-)
    target_compile_options(fobos_sdr_proxy_logger PRIVATE /W4 /permissive-)
endif()
