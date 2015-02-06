#ifndef _RANDOM_ELLIPSE_2_HPP_
#define _RANDOM_ELLIPSE_2_HPP_

#include <CGAL/point_generators_2.h>

// Generate N points on an square with minor / major axes of a / b.
// Pre-condition:
// -> OutputIterator::value_type = Point_2
template <typename OutputIterator>
void random_ellipse_2 (int N,
                      float a,
                      float b,
                      OutputIterator out) {
    typedef CGAL::Random_points_on_circle_2<Point_2> Random_points_on_circle_2;

    Random_points_on_circle_2 random_points(a);
    for (int i = 0; i < N; ++i) {
        Point_2 p = *random_points,
                q(a * p.x(), b * p.y());
        *(out++) = q;
        random_points++;
    }
}

#endif

