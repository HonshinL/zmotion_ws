cmake_minimum_required(VERSION 3.10)
project(zmotion_test)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)
link_directories(lib)

add_executable(zmotion_test
    src/main.cpp
    src/zmcaux.cpp
)

target_link_libraries(zmotion_test
    zmotion
    pthread
)
