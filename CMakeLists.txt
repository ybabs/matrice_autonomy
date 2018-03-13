#required cmake version
cmake_minimum_required(VERSION 2.8.3)
#project name
project(matrice_autonomy)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  dji_sdk
  joy
)

find_package(DJIOSDK REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
  
  CATKIN_DEPENDS joy
)

#Specify include directories for zee packages
include_directories(include ${catkin_INCLUDE_DIRS})

# specify executables for the catkin package
add_executable(xbox_control src/xbox_control.cpp)

target_link_libraries(xbox_control ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})


add_dependencies(xbox_control dji_sdk_generate_messages_cpp)

# specify executables for the sensor data collection package
#add_executable(flight_data src/flight_data.cpp)

#target_link_libraries(flight_data ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})

#add_dependencies(flight_data dji_sdk_generate_messages_cpp)

# specify executables for the sensor data collection package
add_executable(mission_control src/mission_control.cpp src/flight_data.cpp  src/mobile_comm.cpp)

target_link_libraries(mission_control ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})

add_dependencies(mission_control dji_sdk_generate_messages_cpp)

## Mark cpp header files for installation

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
