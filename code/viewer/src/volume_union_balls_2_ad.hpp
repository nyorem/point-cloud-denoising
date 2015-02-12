#ifndef _VOLUME_UNION_BALLS_2_AD_H_
#define _VOLUME_UNION_BALLS_2_AD_H_

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Bbox_2.h>

#include <cmath>
#include <vector>

#include <iostream>

// Sort points in a counter clockwise order
// wrt a reference point.
template <typename InputIterator, typename Point>
void sort_counter_clockwise (InputIterator begin,
                             InputIterator beyond,
                             Point ref) {
    struct CompareAngle {
        CompareAngle (Point ref) : m_ref(ref) {}

        bool operator() (Point p, Point q) {
            double thetap = std::atan2(p.y().value() - m_ref.y().value(), p.x().value() - m_ref.x().value()),
                   thetaq = std::atan2(q.y().value() - m_ref.y().value(), q.x().value() - m_ref.x().value());

            return thetap > thetaq;
        }

        private:
            Point m_ref;
    } compareAngle(ref);

    std::sort(begin, beyond, compareAngle);
}

// Intersection between a segment and a sphere
template <class Point, class Segment, class OutputIterator>
bool segment_sphere_intersect (Point o, double r,
                               Segment seg,
                               OutputIterator out) {
    typedef typename CGAL::Kernel_traits<Point>::Kernel Kernel;
    typedef typename Kernel::Vector_2 Vector;
    typedef typename Kernel::FT FT;

    Point s = seg.source();
    Vector v = seg.target() - seg.source(),
           e = s - o;

    FT a = v.squared_length(),
       b = 2 * v * e,
       c = e.squared_length() - r * r;

    FT delta = b * b - 4 * a * c;

    if (delta < 0) {
        return false;
    }

    bool result = false;
    const double pm[] = {+1.0, -1.0};
    for (size_t i = 0; i < 2; ++i) {
        FT ti = (-b + pm[i] * sqrt(delta)) / (2 * a);
        Point pi = s + ti * v;
        if (ti >= 0 && ti <= 1) {
            *out++ = pi;
            result = true;
        }
    }

    return result;
}

// Positive floating modulus
double fmodpos (double x, double N) {
    return std::fmod(std::fmod(x, N) + N, N);
}

// Area of an angular sector defined by the vectors op and oq
template <typename Vector>
double angular_sector_area (Vector op, Vector oq,
                            double radius) {
    if (op == Vector(0, 0) || oq == Vector(0, 0)) {
        return 0;
    }

    double theta1 = fmodpos(std::atan2(op.y().value(), op.x().value()), 2 * M_PI),
           theta2 = fmodpos(std::atan2(oq.y().value(), oq.x().value()), 2 * M_PI);
    double angle = fmodpos(theta2 - theta1, 2 * M_PI);

    return radius * radius * angle / 2;
}

// Perimeter of an angular sector defined by the vectors op and oq
template <typename Vector>
double perimeter_sector_area (Vector op, Vector oq,
                              double radius) {
    if (op == Vector(0, 0) || oq == Vector(0, 0)) {
        return 0;
    }

    double theta1 = fmodpos(std::atan2(op.y().value(), op.x().value()), 2 * M_PI),
           theta2 = fmodpos(std::atan2(oq.y().value(), oq.x().value()), 2 * M_PI);
    double angle = fmodpos(theta2 - theta1, 2 * M_PI);

    return radius * angle;
}

// Compute the volume of the r-offset of the point cloud
// InputIterator::value_type = Point
template <typename FT, typename InputIterator>
FT volume_union_balls_2 (InputIterator begin,
                         InputIterator beyond,
                         double radius) {
    typedef typename std::iterator_traits<InputIterator>::value_type Point;
    typedef typename CGAL::Kernel_traits<Point>::Kernel Kernel;
    typedef typename Kernel::Segment_2 Segment;
    typedef typename Kernel::Line_2 Line;
    typedef typename CGAL::Delaunay_triangulation_2<Kernel> DT;

    // Bounding box
    CGAL::Bbox_2 b = CGAL::bbox_2(begin, beyond);
    Point bl(b.xmin() - 2 * radius, b.ymin() - 2 * radius),
          br(b.xmax() + 2 * radius, b.ymin() - 2 * radius),
          tl(b.xmin() - 2 * radius, b.ymax() + 2 * radius),
          tr(b.xmax() + 2 * radius, b.ymax() + 2 * radius);

    // Delaunay triangulation
    DT dt(begin, beyond);
    dt.insert(bl); dt.insert(br);
    dt.insert(tl); dt.insert(tr);

    FT vol = 0;

    // Decomposition of the intersection of a Voronoi cell and a ball
    // It is made of triangles and agular sectors
    for (typename DT::Finite_vertices_iterator vit = dt.finite_vertices_begin();
         vit != dt.finite_vertices_end();
         ++vit) {
        Point P = vit->point();
        if (P == bl || P == br || P == tl || P == tr) {
            continue;
        }

        // Compute the boundary of the Voronoi cell of P
        typename DT::Face_circulator fc = dt.incident_faces(vit), done(fc);
        std::vector<Point> adjacent_voronoi_vertices;
        std::cout << "Voronoi vertices of: " << P << std::endl;
        do {
            std::cout << dt.dual(fc) << std::endl;
            adjacent_voronoi_vertices.push_back(dt.dual(fc));
        } while (++fc != done);

        // Test if the first Voronoi vertex is inside
        Point p0 = adjacent_voronoi_vertices[0];
        bool isInside = (p0 - P).squared_length() <= radius * radius;
        bool allOutside = true;

        const int N = adjacent_voronoi_vertices.size();

        // Voronoi edges
        std::vector<Segment> voronoi_edges;
        for (int i = 0; i < N; ++i) {
            voronoi_edges.push_back(Segment(adjacent_voronoi_vertices[i],
                                            adjacent_voronoi_vertices[(i + 1) % N]));
        }

        std::vector<Point> boundary;
        std::map<Point, Segment> edge_map;
        std::map<Point, bool> interior_map;

        // For each Voronoi vertex
        for (int i = 0; i < N; ++i) {
            Point p = adjacent_voronoi_vertices[i],
                  next = adjacent_voronoi_vertices[(i + 1) % N];
            Segment edge(p, next);

            if (isInside) {
                boundary.push_back(p);
            }
            interior_map[p] = isInside;

            // Intersection points between the Voronoi edge [p, next] and the ball
            std::cout << "Current edge: " << edge << std::endl;
            std::vector<Point> inter;
            segment_sphere_intersect(P, radius, edge, std::back_inserter(inter));
            std::cout << "Size intersection: " << inter.size() << std::endl;

            // Decompose the intersection accordingly
            if (inter.size() != 0) {
                for (size_t i = 0; i < inter.size(); ++i) {
                    edge_map[inter[i]] = edge;
                }
                boundary.insert(boundary.end(), inter.begin(), inter.end());
                allOutside = false;
            }

            // Update isInside boolean
            std::cout << "isInside = " << (isInside ? "true" : "false") << std::endl;
            if (inter.size() == 1) {
                isInside = !isInside;
            }
        }

        // Sort the points in a counter clowkise order
        sort_counter_clockwise(boundary.begin(), boundary.end(), P);
        std::cout << "Boundary" << std::endl;
        for (size_t i = 0; i < boundary.size(); ++i) {
            std::cout << boundary[i] << std::endl;
        }
        std::cout << std::endl;

        // The boundary of the Voronoi cell is entirely outside of the ball
        std::cout << "allOutside " << allOutside << std::endl;
        if (allOutside) {
            vol += M_PI * radius * radius;
            continue;
        }

        // Special case: 2 points
        if (boundary.size() == 2) {
            Point p = boundary[0], pp = boundary[1];
            vol += CGAL::abs(CGAL::area(P, p, pp));
            Line l(p, pp);
            if (l.oriented_side(P) == CGAL::ON_POSITIVE_SIDE) {
                vol += angular_sector_area(pp - P, p - P, radius);
            } else {
                vol += angular_sector_area(p - P, pp - P, radius);
            }

            continue;
        }

        for (size_t i = 0; i < boundary.size(); ++i) {
            Point p = boundary[i],
                  pp = boundary[(i + 1) % boundary.size()];

            if (interior_map[p] && interior_map[pp]) {
                // 2 interior points: triangle
                vol += CGAL::abs(CGAL::area(P, p, pp));
            } else if (interior_map[p] && !interior_map[pp]) {
                // 1 interior point: triangle
                vol += CGAL::abs(CGAL::area(P, p, pp));
            } else if (!interior_map[p] && interior_map[pp]) {
                // 1 interior point: triangle
                vol += CGAL::abs(CGAL::area(P, p, pp));
            } else {
                // 0 interior points: 2 on the boundary
                // It depends on the corresponding edges
                Segment pedge = edge_map[p],
                        ppedge = edge_map[pp];

                if (pedge == ppedge) {
                    // Same edge: triangle
                    vol += CGAL::abs(CGAL::area(P, p, pp));
                } else {
                    // Different edges: angular sector
                    vol += angular_sector_area(p - P, pp - P, radius);
                }
            }
        }

        std::cout << std::endl;
    }

    return vol;
}

// Converts a point cloud to a vector
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

// Converts a vector to a list of points
template <typename Point, typename Vector, typename OutputIterator>
void vectorToPointCloud (Vector const& v,
                         OutputIterator out) {
    int N = v.rows() / 2;
    for (int i = 0; i < N; ++i) {
        *out++ = Point(v(2 * i), v(2 * i + 1));
    }
}

// A version of `volume_union_balls_2` for Eigen vectors
template <typename FT, typename Point, typename Vector>
FT volume_union_balls_2_vector (Vector const& v,
                                double radius) {
    std::vector<Point> vec;
    vectorToPointCloud<Point>(v, std::back_inserter(vec));

    return volume_union_balls_2<FT>(vec.begin(), vec.end(), radius);
}

template <typename PointAD, typename FTAD, typename VectorAD>
struct VolumeUnion {
    VolumeUnion (double radius) : m_radius(radius) {
    }

    FTAD operator() (VectorAD const& v) {
        return volume_union_balls_2_vector<FTAD, PointAD, VectorAD>(v, m_radius);
    }

    private:
        double m_radius;
};

#endif

