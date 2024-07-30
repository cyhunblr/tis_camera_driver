cmake_minimum_required(VERSION 3.5)
project(tis_camera_driver)

# enable C++17 standard (current directory scope)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# configure build types and build c-flags
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")


set(WITH_TONEMAPPING_FLAG OFF)

if(WITH_TONEMAPPING_FLAG)
  message(WARNING "Build with Tonemapping")
  add_definitions( -DTONEMAPPING)
  set(WITH_TONEMAPPING cvl)
else()
  message(WARNING "Build WITHOUT Tonemapping")
endif()


set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ")

add_subdirectory(tiscamera_interface)

find_package(Threads REQUIRED)
find_package(OpenCV 4 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
  image_geometry
)

catkin_package(
  CATKIN_DEPENDS image_geometry roscpp sensor_msgs
  DEPENDS OpenCV
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(tis_camera src/tis_camera.cpp)

target_link_libraries(tis_camera
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  tiscamera_interface::tiscamera_interface
  Threads::Threads
  ${WITH_TONEMAPPING}
)

target_include_directories(tis_camera
  PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
)

