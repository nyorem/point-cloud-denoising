#ifndef _RANDOM_ELLIPSE_2_HPP_
#define _RANDOM_ELLIPSE_2_HPP_

#include <CGAL/point_generators_2.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include "Consts.hpp"

// Generate N points on an ellipse with minor / major axes of a / b.
// Noise can also be added.
// Pre-condition:
// -> OutputIterator::value_type = Point_2
template <typename OutputIterator>
void random_ellipse_2 (int N,
                      float a,
                      float b,
                      float noiseVariance,
                      OutputIterator out) {
    typedef CGAL::Random_points_on_circle_2<Point_2> Random_points_on_circle_2;

    boost::normal_distribution<float> nd(0.0, std::sqrt(noiseVariance));
    boost::variate_generator<boost::mt19937, boost::normal_distribution<float> > gen(consts::g_eng, nd);

    Random_points_on_circle_2 random_points(a);
    for (int i = 0; i < N; ++i) {
        double noise = gen();

        Point_2 p = *random_points,
                q(a * p.x() + noise, b * p.y() + noise);
        *(out++) = q;
        random_points++;
    }
}

#endif

