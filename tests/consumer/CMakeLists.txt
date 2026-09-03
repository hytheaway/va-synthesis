cmake_minimum_required(VERSION 3.22)
project(va_synthesis_consumer LANGUAGES CXX)

find_package(va-synthesis CONFIG REQUIRED)

add_executable(va_synthesis_consumer main.cpp)
target_link_libraries(va_synthesis_consumer PRIVATE va::synthesis)
