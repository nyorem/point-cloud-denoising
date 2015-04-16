#ifndef _RANDOM_SPHERE_3_CPP_
#define _RANDOM_SPHERE_3_CPP_

#include <iterator>
#include <CGAL/point_generators_3.h>

// Random double between 'a' and 'b'
static inline double rr (double a, double b) {
    float random = ((double) rand()) / (double) RAND_MAX;

    return a + random * (b - a);
}

// Generate random points on a sphere (uniformy or not).
template < typename Point
, typename OutputIterator
>
void random_sphere_3 (size_t N,
                      double r,
                      bool uniform,
                      OutputIterator out) {
    if (uniform) {
        for (int i = 0; i < N; ++i) {
            double z = rr(-1.0, 1.0);
            double theta = rr(-M_PI, M_PI);
            double x = sin(theta) * sqrt(1 - z * z);
            double y = cos(theta) * sqrt(1 - z * z);
            *out++ = Point(x, y, z);
        }
    } else {
        CGAL::Random_points_on_sphere_3<Point> g(r);

        for (int i = 0; i < N; ++i) {
            *out++ = *g;
            g++;
        }
    }
}

// Generate random points on an ellispoid (uniformly or not).
template < typename Point
, typename OutputIterator
>
void random_ellipsoid_3 (size_t N,
                         double a, double b, double c,
                         bool uniform,
                         OutputIterator out) {
    std::vector<Point> points;
    random_sphere_3<Point>(N, 1.0, uniform, std::back_inserter(points));

    for (int i = 0; i < N; ++i) {
        Point p(a * points[i].x(),
                b * points[i].y(),
                c * points[i].z());
        *out++ = p;
    }
}

#endif

