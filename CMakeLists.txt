cmake_minimum_required(VERSION 2.8.4)
project(elmo C)

option(EXAMPLE "build example" OFF)

add_subdirectory(SOEM)
add_subdirectory(src)

if(EXAMPLE)
  add_subdirectory(example)
endif(EXAMPLE)
