cmake_minimum_required(VERSION 2.8.11)

project(3dviewer)

set (EXECUTABLE_NAME "main")
set (SOURCE_DIR "src")
set (INCLUDE_DIR "src")

# Compilation flags
add_definitions (-Wall)

# Modules
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Qt5
if (MAC)
    set (CMAKE_PREFIX_PATH "/usr/local/Cellar/qt5/5.4.2/")
endif (MAC)
set (CMAKE_AUTOMOC ON)
set (CMAKE_INCLUDE_CURRENT_DIR ON)

## Components
find_package(Qt5Core REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Xml REQUIRED)

# CGAL with Qt5
find_package(CGAL REQUIRED COMPONENTS Qt5)
include (${CGAL_USE_FILE})

# Boost
find_package(Boost REQUIRED)

# Eigen3
find_package (Eigen3 REQUIRED)

# QGLViewer
find_package (QGLViewer REQUIRED)

# OpenGL
find_package (OpenGL REQUIRED)

# Include directories
include_directories(${INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

# ui
qt5_wrap_ui (VIEWER_UI_HEADERS polyhedron.ui)

# resources
qt5_add_resources (RESOURCE_FILES polyhedron.qrc)

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
ADD_EXECUTABLE(${EXECUTABLE_NAME}
    ${${EXECUTABLE_NAME}_SOURCES}
    ${VIEWER_UI_HEADERS}
)

# Libraries
target_link_libraries(${EXECUTABLE_NAME}
    ${OPENGL_gl_LIBRARY}
    ${OPENGL_glu_LIBRARY}
    ${QGLVIEWER_LIBRARIES}
    Qt5::Core
    Qt5::OpenGL
    Qt5::Widgets
    Qt5::Xml
)
