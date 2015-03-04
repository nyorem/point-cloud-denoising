#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <Eigen/Dense>

#include <iostream>
#include <vector>

#include "volume_union_balls_2.h"
#include "volume_union_balls_2_debug.h"
#include "ps.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Point_2<Kernel> Point;
typedef CGAL::Segment_2<Kernel> Segment;
typedef Kernel::FT FT;
typedef Eigen::VectorXd VectorXd;

int main (int argc, const char *argv[]) {
    // Test volume_union_balls_2
    std::vector<Point> points;
    points.push_back(Point(0, 0));
    points.push_back(Point(0, 1));
    points.push_back(Point(1, 0));
    points.push_back(Point(2, 0));
    points.push_back(Point(1.5, 0));
    points.push_back(Point(3, 0));
    /* points.push_back(Point(-88.5358, 0.71007)); */
    /* points.push_back(Point(-88.4987, 1.32109)); */
    /* points.push_back(Point(-87.269, 0.793098)); */
    /* points.push_back(Point(-81.3591, 0.859669)); */
    /* points.push_back(Point(-81.3373, 1.31607)); */
    ps_header(std::cerr);
    std::cout << volume_union_balls_2<FT>(points.begin(), points.end(), 1) << std::endl;
    ps_footer(std::cerr);

    // Test volume_union_balls_2_debug
    /* std::vector<Segment> segments; */
    /* std::cout << volume_union_balls_2_debug<FT>(points.begin(), points.end(), 1, segments) << std::endl; */
    /* std::cout << segments.size() << std::endl; */

    // Test volume_union_balls_2_vector_out
    /* VectorXd vol = volume_union_balls_2_vector_out<VectorXd>(points.begin(), points.end(), 1); */
    /* std::cout << vol << std::endl; */

    // Test volume_union_balls_2_vector_in
    /* VectorXd points_vec(2 * 2); */
    /* points_vec[0] = points[0].x(); */
    /* points_vec[1] = points[0].y(); */
    /* points_vec[2] = points[1].x(); */
    /* points_vec[3] = points[1].y(); */
    /* std::cout << volume_union_balls_2_vector_in<FT, Point>(points_vec, 1) << std::endl; */

    // Test in_counter_clockwise
    /* std::cout << in_counter_clockwise(Point(1, -1), Point (1, 1), Point(0, 0)) << std::endl; */
    /* std::cout << in_counter_clockwise(Point(1, 1), Point (1, -1), Point(0, 0)) << std::endl; */
    /* std::cout << in_counter_clockwise(Point(2, 0), Point (2, 2), Point(1, 1)) << std::endl; */
    /* std::cout << in_counter_clockwise(Point(2, 2), Point (2, 0), Point(1, 1)) << std::endl; */
    /* std::cout << in_counter_clockwise(Point(0.75, -0.6), Point (0.75, 0.6), Point(1.5, 0)) << std::endl; */
    /* std::cout << in_counter_clockwise(Point(0.75, -0.6), Point (0.75, 0.6), Point(1.5, 0)) << std::endl; */

    return 0;
}

