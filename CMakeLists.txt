cmake_minimum_required(VERSION 3.10)
project(ME553_2024)

set(CMAKE_CXX_STANDARD 17)
include(cmake/helper.cmake)

# exercise
create_executable(exercise1 src/exercise1.cpp)
create_executable(exercise2 src/exercise2.cpp)
create_executable(exercise3 src/exercise3.cpp)
create_executable(exercise4 src/exercise4.cpp)
create_executable(exercise5 src/exercise5.cpp)

# examples
create_executable(rotation_example_angle_axis src/rotation_example_angle_axis.cpp)
create_executable(rotation_example_euler_angles src/rotation_example_euler_angles.cpp)


