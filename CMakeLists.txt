cmake_minimum_required(VERSION 2.8.4)
project(ec_elmo C)

add_subdirectory(SOEM)

include_directories(include)
include_directories(SOEM)

aux_source_directory(./src DIR_SRCS)
add_library(${PROJECT_NAME} STATIC
  ${DIR_SRCS}
)
target_include_directories(${PROJECT_NAME} PUBLIC ./include)
target_link_libraries(${PROJECT_NAME} soem)

add_subdirectory(test)
