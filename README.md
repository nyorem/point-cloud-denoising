# Adaptive point cloud denoising

## Organization of the repository:

* code:
	* viewer: 2D point cloud denoising with a discrete Mean Curvature Flow approach
	* 3dviewer: 3D point cloud anistropic denoising
	* python: python bindings for the 2D case
* report: final master report
* pres: slides for the presentations (team meeting and oral defense)

## Dependencies

* CGAL
* Eigen
* CMake
* Qt4 or Qt5

Python bindings:

* Boost.Python
* Boost.NumPy (`git submodule update --init`)
* NumPy, SciPy, matplotlib

## Building the code

`mkdir build && cd build && cmake .. && make`
