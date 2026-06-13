cmake_minimum_required(VERSION 3.16)

project(rtlsdr_proxy_logger LANGUAGES CXX)

add_library(rtlsdr_proxy_logger SHARED rtlsdr_proxy_logger.cpp)
target_compile_features(rtlsdr_proxy_logger PRIVATE cxx_std_17)
set_target_properties(rtlsdr_proxy_logger PROPERTIES
    OUTPUT_NAME rtlsdr
    PREFIX ""
)

if(MSVC)
    target_compile_options(rtlsdr_proxy_logger PRIVATE /W4 /permissive-)
endif()
