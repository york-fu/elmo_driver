cmake_minimum_required(VERSION 2.8.4)
project(ecm_elmo_example C)

include_directories(.)
aux_source_directory(. EXAMPLE_SRCS)

add_executable(${PROJECT_NAME} ${EXAMPLE_SRCS})
target_link_libraries(${PROJECT_NAME} ecm_elmo -lm)
