cmake_minimum_required(VERSION 2.8.4)
project(elmo_driver C)

option(BUIDL_EXAMPLE "build example" OFF)

add_subdirectory(SOEM)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/SOEM)
aux_source_directory(./src DIR_SRCS)

add_library(${PROJECT_NAME} STATIC ${DIR_SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC ./SOEM ./include)
target_link_libraries(${PROJECT_NAME} soem)

if(BUIDL_EXAMPLE)
  add_subdirectory(test)
endif(BUIDL_EXAMPLE)
