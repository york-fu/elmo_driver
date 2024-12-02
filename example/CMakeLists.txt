cmake_minimum_required(VERSION 2.8.4)
project(ecm_elmo_example C)

include_directories(.)

add_executable(enable enable.c)
target_link_libraries(enable ecm_elmo -lm)

add_executable(pd_lock pd_lock.c)
target_link_libraries(pd_lock ecm_elmo -lm)

set(SRCS 
  ./util.c
)
add_executable(traj_ctrl ${SRCS} traj_ctrl.c)
target_link_libraries(traj_ctrl ecm_elmo -lm)

set(SRCS 
  ./util.c
)
add_executable(sampling ${SRCS} sampling.c)
target_link_libraries(sampling ecm_elmo -lm)

set(SRCS 
  ./util.c
)
add_executable(cascade_ctrl ${SRCS} cascade_ctrl.c)
target_link_libraries(cascade_ctrl ecm_elmo -lm)
