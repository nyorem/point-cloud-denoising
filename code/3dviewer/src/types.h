#ifndef TYPES_H
#define TYPES_H

// Epick
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

// Types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Enriched_polyhedron<Kernel, Enriched_items> Polyhedron;
typedef CPoint_cloud<Kernel> Point_cloud;
typedef CVector_field<Kernel> Vector_field;

typedef CGAL::Bbox_3 Bbox;

// AD
#include <CGAL/Simple_cartesian.h>
#include "CGAL_AD.hpp"

typedef CGAL::Simple_cartesian<AD> Kernel_ad;
typedef Kernel_ad::FT FT_ad;
typedef CGAL::Point_3<Kernel_ad> Point_ad;
typedef CGAL::Vector_3<Kernel_ad> Vector_ad;

typedef Eigen::Matrix<FT_ad, Eigen::Dynamic, 1> VectorXd_ad;
typedef Eigen::VectorXd VectorXd;

#include "minkowski_sum_pointcloud_convex_polyhedron_3.h"

typedef Signed_volume_polyhedron_3<FT_ad> Volume_polyhedron_ad;
typedef UnionPolyhedra<Point_ad, FT_ad, VectorXd_ad,
                       Volume_polyhedron_ad> FunctionUnion_ad;

#endif // TYPES_H

