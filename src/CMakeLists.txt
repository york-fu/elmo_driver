cmake_minimum_required(VERSION 2.8.4)

include_directories(./)

set(SRCS 
  ./ec_elmo.c
  ./elmo_motor.c
)
add_library(${PROJECT_NAME} STATIC ${SRCS})
target_link_libraries(${PROJECT_NAME} soem)
target_include_directories(${PROJECT_NAME} PUBLIC ./)

