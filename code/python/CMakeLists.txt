project(python-unionballs)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Boost + Boost::Python
FIND_PACKAGE(Boost 1.54.0 COMPONENTS python REQUIRED)
link_libraries(${Boost_LIBRARIES})
INCLUDE_DIRECTORIES("${Boost_INCLUDE_DIRS}")

# enable C++11
ADD_DEFINITIONS(-Wall -std=c++11)

# python
FIND_PACKAGE(PythonInterp)
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
link_libraries(${PYTHON_LIBRARIES})

# Find the correct python library
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    SET(PYTHON_LIBRARY /usr/local/Cellar/python/2.7.9/Frameworks/Python.framework/Versions/2.7/lib/libpython2.7.dylib CACHE STRING "" FORCE)
endif ()

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# boost numpy
ADD_SUBDIRECTORY(thirdparty/Boost.NumPy)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/thirdparty/Boost.NumPy)
LINK_LIBRARIES(boost_numpy)

# CGAL and its components
find_package(CGAL REQUIRED COMPONENTS)
include( ${CGAL_USE_FILE} )
LINK_LIBRARIES(${CGAL_LIBRARIES})

ADD_LIBRARY(unionballs MODULE unionballs.cpp)
SET_TARGET_PROPERTIES(unionballs PROPERTIES PREFIX "")

