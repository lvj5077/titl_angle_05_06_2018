
include_directories("/home/robot/catkin_ws/src/imu_vn100/src")

add_library(intend_control motor_intend.cpp user_model.cpp user_model_discrete.cpp)
target_link_libraries(intend_control control vn100_reader )
add_executable(test_motor_intend test_motor_intend.cpp)