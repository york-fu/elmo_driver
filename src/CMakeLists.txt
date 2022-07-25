cmake_minimum_required(VERSION 2.8.4)

include_directories(./)
aux_source_directory(./ DIR_SRCS)

add_library(${PROJECT_NAME} STATIC ${DIR_SRCS})
target_link_libraries(${PROJECT_NAME} soem)
target_include_directories(${PROJECT_NAME} PUBLIC ./)

