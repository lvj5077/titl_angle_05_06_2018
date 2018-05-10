project(tilt_angle)

cmake_minimum_required(VERSION 3.5)
set (CMAKE_CXX_STANDARD 11)

include_directories("/usr/include/eigen3")

# FIND_PACKAGE(Eigen3 REQUIRED)



# IF(EIGEN3_FOUND)
#   message(STATUS "EIGEN3_FOUND")
# ENDIF(EIGEN3_FOUND)


include_directories("/home/robot/catkin_ws/src/imu_vn100/src")



add_executable(tiltAngle tiltAngle.cpp)
target_link_libraries(tiltAngle /home/robot/catkin_ws/devel/lib/libvn100_reader.so)

