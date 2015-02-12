#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <iostream>
#include <vector>

#include "volume_union_balls_2.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Point_2<Kernel> Point;
typedef Kernel::FT FT;

int main (int argc, const char *argv[]) {
    std::vector<Point> points;
    points.push_back(Point(0, 0));
    points.push_back(Point(1.5, 0));
    points.push_back(Point(3, 0));
    /* points.push_back(Point(2, 0)); */
    /* points.push_back(Point(1, 1)); */
    std::cout << volume_union_balls_2<FT>(points.begin(), points.end(), 1) << std::endl;

    return 0;
}

