#ifndef _CGAL_AD_TYPEDEFS_HPP_
#define _CGAL_AD_TYPEDEFS_HPP_

#include "CGAL_AD.hpp"
#include "volume_union_balls_2_ad.hpp"

// AD
typedef CGAL::Simple_cartesian<AD> Kernel_ad;
typedef Kernel_ad::FT FT_ad;
typedef CGAL::Point_2<Kernel_ad> Point_ad;

typedef Eigen::Matrix<FT_ad, Eigen::Dynamic, 1> VectorXd_ad;
typedef std::function<FT_ad(const VectorXd_ad &x)> Function_ad;
typedef VolumeUnion<Point_ad, FT_ad, VectorXd_ad> VolumeUnion_ad;

#endif

