#ifndef _CGAL_AD_TYPEDEFS_HPP_
#define _CGAL_AD_TYPEDEFS_HPP_

#include "CGAL_AD.hpp"
#include "union_balls_2.hpp"

// AD
typedef CGAL::Simple_cartesian<AD> Kernel_ad;
typedef Kernel_ad::FT FT_ad;
typedef CGAL::Point_2<Kernel_ad> Point_ad;

typedef Eigen::Matrix<FT_ad, Eigen::Dynamic, 1> VectorXd_ad;
typedef std::function<FT_ad(const VectorXd_ad &x)> Function_ad;

// Perimeter
typedef UnionBalls<Point_ad, FT_ad, VectorXd_ad,
                   Accumulator<FT_ad>,
                   AngularSectorPerimeterAccumulator<FT_ad> > PerimeterBoundaryUnion_ad;

// Area
typedef UnionBalls<Point_ad, FT_ad, VectorXd_ad,
                   TriangleAreaAccumulator<FT_ad>,
                   AngularSectorAreaAccumulator<FT_ad> > AreaUnion_ad;

// Area weigthed by the perimeter
/* typedef WeightedUnionBalls<Point_ad, FT_ad, VectorXd_ad, */
/*                            // compute area */
/*                            TriangleAreaAccumulator<FT_ad>, */
/*                            AngularSectorAreaAccumulator<FT_ad>, */
/*                            // divided by perimeter */
/*                            Accumulator<FT_ad>, */
/*                            AngularSectorPerimeterAccumulator<FT_ad> > FunctionUnion_ad; */

#endif

