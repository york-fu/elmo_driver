cmake_minimum_required(VERSION 2.8.4)
project(elmo_test C)

add_executable(${PROJECT_NAME}
  ./elmo_test.c
)
target_link_libraries(${PROJECT_NAME} ec_elmo -lm)
