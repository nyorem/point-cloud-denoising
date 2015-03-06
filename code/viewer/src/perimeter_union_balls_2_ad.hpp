#ifndef _PERIMETER_UNION_BALLS_2_AD_HPP_
#define _PERIMETER_UNION_BALLS_2_AD_HPP_

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Bbox_2.h>

#include <cmath>
#include <vector>

#include <iostream>

#include "ps.hpp"

// Test if p and q are in a counter clockwise order wrt ref
template <typename Point>
bool in_counter_clockwise (Point const& p, Point const& q,
                           Point const& ref) {
    return CGAL::left_turn(ref, p, q);
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

// Perimeter of an angular sector defined by the vectors op and oq
template <typename Point, typename Vector>
double angular_sector_perimeter (Point P, Vector op, Vector oq,
                                 double radius) {
    if (op == Vector(0, 0) || oq == Vector(0, 0)) {
        return 0;
    }

    double theta1 = fmodpos(std::atan2(op.y().value(), op.x().value()), 2 * M_PI),
           theta2 = fmodpos(std::atan2(oq.y().value(), oq.x().value()), 2 * M_PI);
    double angle = fmodpos(theta2 - theta1, 2 * M_PI);

    ps_arc(std::cerr, P, radius, theta1, theta2, false, 255, 0, 0);

    return radius * angle;
}

// Volume of one ball intersected with a Voronoi cell
template <typename Kernel, typename DT, typename Vertex_handle>
typename Kernel::FT perimeter_ball_voronoi_cell_2 (DT const& dt,
                                                   Vertex_handle const& v,
                                                   double radius) {
    typedef typename Kernel::Point_2 Point;
    typedef typename Kernel::Line_2 Line;
    typedef typename Kernel::FT FT;
    typedef typename Kernel::Segment_2 Segment;

    Point P = v->point();

    // >>>
    FT per = 0;

    // Compute the boundary of the Voronoi cell of P
    typename DT::Face_circulator fc = dt.incident_faces(v), done(fc);
    std::vector<Point> adjacent_voronoi_vertices;
    std::cout << "Voronoi vertices of: " << P << std::endl;
    do {
        std::cout << dt.dual(fc) << std::endl;
        adjacent_voronoi_vertices.push_back(dt.dual(fc));
    } while (++fc != done);

    // Test if the first Voronoi vertex is inside
    Point p0 = adjacent_voronoi_vertices[0];
    bool isInside = (p0 - P).squared_length() <= radius * radius;
    bool allOutside = !isInside;

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

        // Remember the edge corresponding to an intersection point
        if (inter.size() != 0) {
            for (size_t i = 0; i < inter.size(); ++i) {
                edge_map[inter[i]] = edge;
            }
            if (inter.size() == 2) {
                // Keep a counter clockwise order
                if (in_counter_clockwise(inter[0], inter[1], P)) {
                    boundary.insert(boundary.end(), inter.begin(), inter.end());
                } else {
                    boundary.push_back(inter[1]);
                    boundary.push_back(inter[0]);
                }
            } else {
                boundary.insert(boundary.end(), inter.begin(), inter.end());
            }
            allOutside = false;
        }

        // Update isInside boolean
        std::cout << "isInside = " << (isInside ? "true" : "false") << std::endl;
        if (inter.size() == 1) {
            isInside = !isInside;
        }
    }

    // The boundary of the Voronoi cell is entirely outside of the ball
    std::cout << "allOutside " << allOutside << std::endl;
    if (allOutside) {
        ps_arc(std::cerr, P, radius, 0, 2 * M_PI, true, 255, 0, 0);

        per = 2 * M_PI * radius;
        std::cout << "perimeter: " << per << std::endl;
        return per;
    }

    // Special case: 2 points
    if (boundary.size() == 2 && !interior_map[boundary[0]] && !interior_map[boundary[1]]) {
        Point p = boundary[0], pp = boundary[1];
        Line l(p, pp);
        // Only 1 point: balls are tangential
        if (p == pp) {
            ps_arc(std::cerr, P, radius, 0, 2 * M_PI, true, 255, 0, 0);

            per = 2 * M_PI * radius;
            std::cout << "perimeter: " << per << std::endl;

            return per;
        }

        if (l.oriented_side(P) == CGAL::ON_POSITIVE_SIDE) {
            ps_triangle(std::cerr, P, p, pp);

            per += angular_sector_perimeter(P, pp - P, p - P, radius);
            std::cout << "perimeter: " << per << std::endl;
        } else {
            ps_triangle(std::cerr, P, pp, p);

            per += angular_sector_perimeter(P, p - P, pp - P, radius);
            std::cout << "perimeter: " << per << std::endl;
        }

        return per;
    }

    for (size_t i = 0; i < boundary.size(); ++i) {
        Point p = boundary[i],
              pp = boundary[(i + 1) % boundary.size()];

        if (interior_map[p] && interior_map[pp]) {
            // 2 interior points: triangle
            ps_triangle(std::cerr, P, p, pp);
        } else if (interior_map[p] && !interior_map[pp]) {
            // 2 interior points: triangle
            ps_triangle(std::cerr, P, p, pp);
        } else if (!interior_map[p] && interior_map[pp]) {
            // 2 interior points: triangle
            ps_triangle(std::cerr, P, p, pp);
        } else {
            // 0 interior points: 2 on the boundary
            // It depends on the corresponding edges
            Segment pedge = edge_map[p],
                    ppedge = edge_map[pp];

            if (pedge == ppedge) {
                // Same edge: triangle
                ps_triangle(std::cerr, P, p, pp);
            } else {
                // Different edges: angular sector
                per += angular_sector_perimeter(P, p - P, pp - P, radius);
            }
        }
    }

    return per;
}

// Compute the volume of each Voronoi cell intersected with a ball
// Put all the results in a Eigen vector
template <typename Vector, typename InputIterator>
Vector perimeter_union_balls_2_vector_out (InputIterator begin,
                                           InputIterator beyond,
                                           double radius) {
    typedef typename std::iterator_traits<InputIterator>::value_type Point;
    typedef typename CGAL::Kernel_traits<Point>::Kernel Kernel;
    typedef typename CGAL::Delaunay_triangulation_2<Kernel> DT;

    // Bounding box
    CGAL::Bbox_2 b = CGAL::bbox_2(begin, beyond);
    // >>>
    Point bl(b.xmin() - 2 * radius, b.ymin() - 2 * radius),
          br(b.xmax() + 2 * radius, b.ymin() - 2 * radius),
          tl(b.xmin() - 2 * radius, b.ymax() + 2 * radius),
          tr(b.xmax() + 2 * radius, b.ymax() + 2 * radius);

    // Delaunay triangulation
    DT dt(begin, beyond);
    dt.insert(bl); dt.insert(br);
    dt.insert(tl); dt.insert(tr);

    int N = 0;
    for (InputIterator it = begin; it != beyond; ++it)
        ++N;
    Vector per(N);

    int i = 0;
    // Decomposition of the intersection of a Voronoi cell and a ball
    // It is made of triangles and agular sectors
    for (typename DT::Finite_vertices_iterator vit = dt.finite_vertices_begin();
         vit != dt.finite_vertices_end();
         ++vit) {
        Point P = vit->point();

        if (P == bl || P == br || P == tl || P == tr) {
            continue;
        }

        per[i++] = perimeter_ball_voronoi_cell_2<Kernel>(dt, vit, radius);
    }

    return per;
}

// Compute the volume of the r-offset of the point cloud
// InputIterator::value_type = Point
template <typename FT, typename InputIterator>
FT perimeter_union_balls_2 (InputIterator begin,
                            InputIterator beyond,
                            double radius) {
    typedef typename std::iterator_traits<InputIterator>::value_type Point;
    typedef typename CGAL::Kernel_traits<Point>::Kernel Kernel;
    typedef typename CGAL::Delaunay_triangulation_2<Kernel> DT;

    // Bounding box
    CGAL::Bbox_2 b = CGAL::bbox_2(begin, beyond);
    // >>>
    Point bl(b.xmin() - 2 * radius, b.ymin() - 2 * radius),
          br(b.xmax() + 2 * radius, b.ymin() - 2 * radius),
          tl(b.xmin() - 2 * radius, b.ymax() + 2 * radius),
          tr(b.xmax() + 2 * radius, b.ymax() + 2 * radius);

    // Delaunay triangulation
    DT dt(begin, beyond);
    dt.insert(bl); dt.insert(br);
    dt.insert(tl); dt.insert(tr);

    // >>>
    FT per = 0;

    // Decomposition of the intersection of a Voronoi cell and a ball
    // It is made of triangles and agular sectors
    for (typename DT::Finite_vertices_iterator vit = dt.finite_vertices_begin();
         vit != dt.finite_vertices_end();
         ++vit) {
        Point P = vit->point();

        if (P == bl || P == br || P == tl || P == tr) {
            continue;
        }

        per += perimeter_ball_voronoi_cell_2<Kernel>(dt, vit, radius);
    }

    return per;
}

// A version of `volume_union_balls_2` that takes the point
// cloud as an Eigen vector.
template <typename FT, typename Point, typename Vector>
FT perimeter_union_balls_2_vector_in (Vector const& v,
                                      double radius) {
    int N = v.rows() / 2;
    std::vector<Point> vec;
    for (int i = 0; i < N; ++i) {
        vec.push_back(Point(v(2 * i), v(2 * i + 1)));
    }

    return perimeter_union_balls_2<FT>(vec.begin(), vec.end(), radius);
}

// A version of `volume_union_balls_2` that takes the point
// cloud as an Eigen vector and that returns an Eigen vector.
template <typename Point, typename Vector>
Vector perimeter_union_balls_2_vector_in_out (Vector const& v,
                                              double radius) {
    int N = v.rows() / 2;
    std::vector<Point> vec;
    for (int i = 0; i < N; ++i) {
        vec.push_back(Point(v(2 * i), v(2 * i + 1)));
    }

    return perimeter_union_balls_2_vector_out<Vector>(vec.begin(), vec.end(), radius);
}

template <typename PointAD, typename FTAD, typename VectorAD>
struct PerimeterUnion {
    PerimeterUnion (double radius) : m_radius(radius) {
    }

    FTAD operator() (VectorAD const& v) {
        ps_header(std::cerr);
        m_perimeters = perimeter_union_balls_2_vector_in_out<PointAD, VectorAD>(v, m_radius);
        ps_footer(std::cerr);

        FTAD per = 0;
        for (int i = 0; i < m_perimeters.rows(); ++i) {
            per += m_perimeters(i);
        }

        return per;
    }

    Eigen::VectorXd grad () const {
        Eigen::VectorXd g = Eigen::VectorXd::Zero(2 * m_perimeters.rows());

        for (int i = 0; i < m_perimeters.rows(); ++i) {
            if (m_perimeters(i).derivatives().rows() != 0) {
                g += m_perimeters(i).derivatives();
            }
        }

        return g;
    }

    private:
        double m_radius;
        VectorAD m_perimeters;
};

#endif

