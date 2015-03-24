#ifndef _RANDOM_ELLIPSE_2_HPP_
#define _RANDOM_ELLIPSE_2_HPP_

#include <CGAL/point_generators_2.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include "Consts.hpp"

// Generate N points uniformly distributed (or nor) on a circle
// with radius 'r'.
// Pre-condition:
// -> OutputIterator::value_type = Point_2
template <typename OutputIterator>
void random_on_circle_2 (int N,
                         float r,
                         bool uniform,
                         OutputIterator out) {
    if (uniform) {
        for (int i = 0; i < N; ++i) {
            Point_2 p(r * cos(2 * i * M_PI / N), r * sin(2 * i * M_PI / N));
            *(out++) = p;
        }
    } else {
        typedef CGAL::Random_points_on_circle_2<Point_2> Random_points_on_circle_2;

        Random_points_on_circle_2 random_points(r);
        for (int i = 0; i < N; ++i) {
            *(out++) = *random_points;
            random_points++;
        }
    }
}

// Random float between 'a' and 'b'
float rr (float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;

    return a + random * (b - a);
}

// Generate N points uniformly distributed (or not) on an ellipse
// with minor / major axes of a / b.
// Noise and oscillations can be added.
// Pre-condition:
// -> OutputIterator::value_type = Point_2
template <typename OutputIterator>
void random_on_ellipse_2 (int N,
                          float a,
                          float b,
                          float noiseVariance,
                          float oscMagnitude,
                          bool uniform,
                          OutputIterator out) {
    consts::g_eng.seed(static_cast<unsigned int>(std::time(0)));
    boost::normal_distribution<float> nd(0.0, std::sqrt(noiseVariance));
    boost::variate_generator<boost::mt19937, boost::normal_distribution<float> > gen(consts::g_eng, nd);

    Points_2 points;
    random_on_circle_2(N, 1.0, uniform, std::back_inserter(points));

    double Amax = oscMagnitude, T = 10;
    for (size_t i = 0; i < points.size(); ++i) {
        Vector_2 v(points[i].x() / a, points[i].y() / b);
        v = v / CGAL::sqrt(v.squared_length());

        double noise = gen();

        Point_2 p(a * points[i].x() + noise,
                  b * points[i].y() + noise);

        int k = T * i / N;
        double A;
        if (k % 2 == 0) {
            A = Amax;
        } else {
            A = Amax * 2;
        }

        *(out++) = p + v * A * sin(2 * i * M_PI / T);
    }
}

#endif

