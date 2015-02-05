cmake_minimum_required(VERSION 2.6)

project(simple)

set (EXECUTABLE_NAME "main")
set (INCLUDE_DIR "inc")
set (SOURCE_DIR "src")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# External Libraries
# Eigen3
find_package (Eigen3 REQUIRED)

# CGAL
find_package (CGAL REQUIRED)
include(${CGAL_USE_FILE})

# C++11 flags
set (CMAKE_C_COMPILER "gcc-4.9")
set (CMAKE_CXX_COMPILER "g++-4.9")
list (APPEND CMAKE_CXX_FLAGS "-std=c++11")

# Compilation flags
add_definitions (-Wall)

# run
ADD_CUSTOM_TARGET(run
    COMMAND ${EXECUTABLE_NAME}
    DEPENDS ${EXECUTABLE_NAME}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
)

# Source files
FILE(
  GLOB_RECURSE
  ${EXECUTABLE_NAME}_SOURCES
  ${SOURCE_DIR}/**.cpp
  )

# Header files
FILE(
  GLOB_RECURSE
  ${EXECUTABLE_NAME}_HEADERS
  ${SOURCE_DIR}/**.hpp
  )

# Include directories
include_directories(${INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

# Executable
ADD_EXECUTABLE(${EXECUTABLE_NAME}
  ${${EXECUTABLE_NAME}_SOURCES}
  ${${EXECUTABLE_NAME}_HEADERS}
  )

