#ifndef _RANDOM_SQUARE_2_HPP_
#define _RANDOM_SQUARE_2_HPP_

#include <CGAL/point_generators_2.h>

// Generate N points inside a square of side a.
// Pre-condition:
// -> OutputIterator::value_type = Point_2
template <typename OutputIterator>
void random_square_2 (int N,
                      float a,
                      OutputIterator out) {
    typedef CGAL::Random_points_in_square_2<Point_2> Random_points_in_square_2;

    Random_points_in_square_2 random_points(a);
    for (int i = 0; i < N; ++i) {
        *(out++) = *random_points++;
    }
}

#endif

