cmake_minimum_required(VERSION 2.8.11)

project(cgviewer)

set (EXECUTABLE_NAME "main")
set (SOURCE_DIR "src")

# Compilation flags
add_definitions (-Wall)

# Qt5
set (CMAKE_PREFIX_PATH "/usr/local/Cellar/qt5/5.4.0/")
# For automoc to work headers and source files must be in the same directory
set (CMAKE_AUTOMOC ON)
set (CMAKE_INCLUDE_CURRENT_DIR ON)

## Components
find_package(Qt5Widgets REQUIRED)

# CGAL
find_package(CGAL REQUIRED)
include (${CGAL_USE_FILE})

# Boost
find_package(Boost REQUIRED)

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

# Executable
ADD_EXECUTABLE(${EXECUTABLE_NAME} ${${EXECUTABLE_NAME}_SOURCES})

# Libraries
target_link_libraries(${EXECUTABLE_NAME}
    Qt5::Widgets
    )
