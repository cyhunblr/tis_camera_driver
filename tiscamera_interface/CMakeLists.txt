
cmake_minimum_required(VERSION 3.5)
project(tiscamera_interface)

# enable C++17 standard (current directory scope)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# configure build types and build c-flags
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()
message(WARNING "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Dependency from pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)
pkg_check_modules(TCAMLIB REQUIRED tcam)

# Other dependencies
find_package(Threads REQUIRED)

# Create library fro tiscamera_interface
add_library(tiscamera_interface STATIC
  src/tiscamera_interface.cpp
  src/tcamcamera.cpp
)
# Create Alias for better diagnosis messages
add_library(tiscamera_interface::tiscamera_interface ALIAS tiscamera_interface)

include_directories(include/tiscamera_interface)

# include directories
target_include_directories(tiscamera_interface
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${GSTREAMER_INCLUDE_DIRS}
    ${TCAMLIB_INCLUDE_DIRS}
)

# libraries
target_link_libraries(tiscamera_interface 
  PUBLIC
    ${TCAMLIB_LIBRARIES} 
    ${GSTREAMER_LIBRARIES}
)

