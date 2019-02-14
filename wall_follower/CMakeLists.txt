cmake_minimum_required(VERSION 2.8.3)
project(wall_follower)


find_package(catkin_simple REQUIRED)

catkin_simple()
cs_add_executable(wall_follower src/wall_follower.cpp)
cs_add_executable(wall_follower_lab src/wall_follower_lab.cpp)
cs_install()
cs_export()
