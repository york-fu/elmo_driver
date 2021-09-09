cmake_minimum_required(VERSION 2.8.4)
project(ec_elmo C)

add_subdirectory(SOEO)

include_directories(include)
include_directories(SOEM)

aux_source_directory(./src DIR_SRCS)
add_executable(${PROJECT_NAME} 
  ${DIR_SRCS}  
)

target_link_libraries(${PROJECT_NAME} soem)
