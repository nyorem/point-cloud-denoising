#ifndef _POINTCLOUDUTILS_HPP_
#define _POINTCLOUDUTILS_HPP_

#include "AD.hpp"

// Converts a vector to a list of points
template <typename Point, typename Vector, typename OutputIterator>
void vectorToPointCloud (Vector const& v,
                         OutputIterator out) {
    int N = v.rows() / 2;
    for (int i = 0; i < N; ++i) {
        *out++ = Point(v(2 * i), v(2 * i + 1));
    }
}

// Converts a point cloud to an AD vector
template <typename Vector, typename InputIterator>
Vector pointCloudToVector (InputIterator begin,
                           InputIterator beyond) {
    int N = 0;
    for (InputIterator it = begin; it != beyond; ++it)
        ++N;

    Vector vec(2 * N);
    int i = 0;
    for (InputIterator it = begin; it != beyond; ++it) {
        vec(2 * i) = AD(it->x(), 2 * N, 2 * i);
        vec(2 * i + 1) = AD(it->y(), 2 * N, 2 * i + 1);
        ++i;
    }

    return vec;
}

#endif

