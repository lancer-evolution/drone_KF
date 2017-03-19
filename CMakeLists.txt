cmake_minimum_required(VERSION 2.8.3)
project(drone_KF)

# 出力先の設定
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
FIND_PACKAGE(Eigen REQUIRED)

catkin_package(
  DEPENDS Eigen
#  INCLUDE_DIRS include
#  LIBRARIES drone_KF
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)



# create library
ADD_LIBRARY(kalmanfilter SHARED src/kalmanfilter.cpp)

add_executable(imu src/imu.cpp)
target_link_libraries(imu kalmanfilter ${catkin_LIBRARIES})
