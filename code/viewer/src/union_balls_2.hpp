#ifndef _UNION_BALLS_2_HPP_
#define _UNION_BALLS_2_HPP_

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Bbox_2.h>
#include <vector>

#include "union_balls_2_utils.hpp"

// Applies function objects on the intersection of a sphere and a voronoi cell:
// -> one function object for the triangles
// -> one function object for the angular sectors
template < typename Kernel,
           typename FTri,
           typename FArc,
           typename DT,
           typename Vertex_handle
         >
void intersection_sphere_voronoi_cell_2 (FTri &ftri, FArc &farc,
                                         DT const& dt,
                                         Vertex_handle const& v,
                                         double radius) {
    typedef typename Kernel::Point_2 Point;
    typedef typename Kernel::Line_2 Line;
    typedef typename Kernel::FT FT;
    typedef typename Kernel::Segment_2 Segment;

    Point P = v->point();

    // Compute the boundary of the Voronoi cell of P
    typename DT::Face_circulator fc = dt.incident_faces(v), done(fc);
    std::vector<Point> adjacent_voronoi_vertices;
    do {
        adjacent_voronoi_vertices.push_back(dt.dual(fc));
    } while (++fc != done);

    // Test if the first Voronoi vertex is inside
    Point p0 = adjacent_voronoi_vertices[0];
    bool isInside = (p0 - P).squared_length() <= radius * radius;
    bool allOutside = !isInside;

    const int N = adjacent_voronoi_vertices.size();

    // Voronoi edges
    std::vector<Segment> voronoi_edges;
    for (size_t i = 0; i < N; ++i) {
        voronoi_edges.push_back(Segment(adjacent_voronoi_vertices[i],
                                        adjacent_voronoi_vertices[(i + 1) % N]));
    }

    std::vector<Point> boundary;
    std::map<Point, Segment> edge_map;
    std::map<Point, bool> interior_map;

    // For each Voronoi vertex
    for (size_t i = 0; i < N; ++i) {
        Point p = adjacent_voronoi_vertices[i],
              next = adjacent_voronoi_vertices[(i + 1) % N];
        Segment edge(p, next);

        if (isInside) {
            boundary.push_back(p);
        }
        interior_map[p] = isInside;

        // Intersection points between the Voronoi edge [p, next] and the ball
        std::vector<Point> inter;
        segment_sphere_intersect(P, radius, edge, std::back_inserter(inter));

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
        if (inter.size() == 1) {
            isInside = !isInside;
        }
    }

    // The boundary of the Voronoi cell is entirely outside of the ball
    if (allOutside) {
        farc(P, radius);

        return;
    }

    // Special case: 2 points
    if (boundary.size() == 2 && !interior_map[boundary[0]] && !interior_map[boundary[1]]) {
        Point p = boundary[0], pp = boundary[1];
        Line l(p, pp);
        // Only 1 point: balls are tangential
        if (p == pp) {
            farc(P, radius);

            return;
        }

        if (l.oriented_side(P) == CGAL::ON_POSITIVE_SIDE) {
            ftri(P, p, pp);
            farc(P, pp - P, p - P, radius);
        } else {
            ftri(P, pp, p);
            farc(P, p - P, pp - P, radius);
        }

        return;
    }

    for (size_t i = 0; i < boundary.size(); ++i) {
        Point p = boundary[i],
              pp = boundary[(i + 1) % boundary.size()];

        if (interior_map[p] && interior_map[pp]) {
            // 2 interior points: triangle
            ftri(P, p, pp);
        } else if (interior_map[p] && !interior_map[pp]) {
            // 2 interior points: triangle
            ftri(P, p, pp);
        } else if (!interior_map[p] && interior_map[pp]) {
            // 2 interior points: triangle
            ftri(P, p, pp);
        } else {
            // 0 interior points: 2 on the boundary
            // It depends on the corresponding edges
            Segment pedge = edge_map[p],
                    ppedge = edge_map[pp];

            if (pedge == ppedge) {
                // Same edge: triangle
                ftri(P, p, pp);
            } else {
                // Different edges: angular sector
                farc(P, p - P, pp - P, radius);
            }
        }
    }
}

// Accumulates value along the intersection of each Voronoi cell
// intersected by a sphere.
// InputIterator::value_type = Point
template < typename FT,
           typename FTri,
           typename FArc,
           typename InputIterator
         >
void union_balls_2 (FTri &ftri, FArc &farc,
                    InputIterator begin,
                    InputIterator beyond,
                    double radius) {
    typedef typename std::iterator_traits<InputIterator>::value_type Point;
    typedef typename CGAL::Kernel_traits<Point>::Kernel Kernel;
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

    // Decomposition of the intersection of a Voronoi cell and a ball
    // It is made of triangles and agular sectors
    for (typename DT::Finite_vertices_iterator vit = dt.finite_vertices_begin();
         vit != dt.finite_vertices_end();
         ++vit) {
        Point P = vit->point();

        if (P == bl || P == br || P == tl || P == tr) {
            continue;
        }

        intersection_sphere_voronoi_cell_2<Kernel>(ftri, farc, dt, vit, radius);
    }
}

// A version of `union_balls_2` that takes the point
// cloud as an Eigen vector.
template < typename FT,
           typename FTri,
           typename FArc,
           typename Point,
           typename Vector
         >
void union_balls_2_vector_in (FTri &ftri, FArc &farc,
                              Vector const& v,
                              double radius) {
    int N = v.rows() / 2;
    std::vector<Point> vec;
    for (int i = 0; i < N; ++i) {
        vec.push_back(Point(v(2 * i), v(2 * i + 1)));
    }

    union_balls_2<FT>(ftri, farc, vec.begin(), vec.end(), radius);
}

// Version of `union_balls_2` that puts the results for each cell
// in an Eigen vector.
template < typename Vector,
           typename FTri,
           typename FArc,
           typename InputIterator
         >
Vector union_balls_2_vector_out (FTri &ftri, FArc &farc,
                                 InputIterator begin,
                                 InputIterator beyond,
                                 double radius) {
    typedef typename std::iterator_traits<InputIterator>::value_type Point;
    typedef typename CGAL::Kernel_traits<Point>::Kernel Kernel;
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

    int N = 0;
    std::map<Point, unsigned> index_points;
    for (InputIterator it = begin; it != beyond; ++it) {
        index_points[*it] = N;
        ++N;
    }
    Vector val(N);

    // Decomposition of the intersection of a Voronoi cell and a ball
    // It is made of triangles and agular sectors
    for (typename DT::Finite_vertices_iterator vit = dt.finite_vertices_begin();
         vit != dt.finite_vertices_end();
         ++vit) {
        Point P = vit->point();

        if (P == bl || P == br || P == tl || P == tr) {
            continue;
        }

        ftri.reset(); farc.reset();
        intersection_sphere_voronoi_cell_2<Kernel>(ftri, farc, dt, vit, radius);
        val[index_points[P]] = ftri.getValue() + farc.getValue();
    }

    return val;
}

// A version of `union_balls_2_vector_out` which takes the point
// cloud as an Eigen vector and which returns an Eigen vector.
template < typename Point,
           typename Vector,
           typename FTri,
           typename FArc
         >
Vector union_balls_2_vector_in_out (FTri &ftri, FArc &farc,
                                    Vector const& v,
                                    double radius) {
    int N = v.rows() / 2;
    std::vector<Point> vec;
    for (int i = 0; i < N; ++i) {
        vec.push_back(Point(v(2 * i), v(2 * i + 1)));
    }

    return union_balls_2_vector_out<Vector>(ftri, farc,
                                            vec.begin(), vec.end(),
                                            radius);
}

// Using Automatic Differentiation, compute the gradient of values along
// intersections of Voronoi cells and spheres.
template < typename PointAD,
           typename FTAD,
           typename VectorAD,
           typename FTri,
           typename FArc
         >
struct UnionBalls {
    UnionBalls (double radius, bool weighting = false) : m_weighting(weighting), m_radius(radius), ftri(), farc() {
    }

    FTAD operator() (VectorAD const& v) {
        ps_header(std::cerr);
        m_values = union_balls_2_vector_in_out<PointAD, VectorAD>(ftri, farc,
                                                                  v,
                                                                  m_radius);
        ps_footer(std::cerr);

        FTAD val = 0;
        for (int i = 0; i < m_values.rows(); ++i) {
            val += m_values(i);
        }

        return val;
    }

    Eigen::VectorXd grad () const {
        Eigen::VectorXd g = Eigen::VectorXd::Zero(2 * m_values.rows()),
                        w = Eigen::VectorXd::Zero(2 * m_values.rows());

        for (int i = 0; i < m_values.rows(); ++i) {
            if (m_values(i).derivatives().rows() != 0) {
                g += m_values(i).derivatives();
            }
            w[2 * i]     = m_values(i).value();
            w[2 * i + 1] = m_values(i).value();
        }

        // Weighting by the corresponding value
        if (m_weighting) {
            g = g.cwiseQuotient(w);
        }

        return g;
    }

    private:
        bool m_weighting;
        double m_radius;
        VectorAD m_values;

        FTri ftri;
        FArc farc;
};

// Using Automatic Differentiation, compute the weighted gradient of values along
// intersections of Voronoi cells and spheres.
template < typename PointAD,
           typename FTAD,
           typename VectorAD,
           typename FTri,
           typename FArc,
           typename FTriW,
           typename FArcW
         >
struct WeightedUnionBalls {
    WeightedUnionBalls (double radius) : m_radius(radius), ftri(), farc() {
    }

    FTAD operator() (VectorAD const& v) {
        ps_header(std::cerr);
        m_values = union_balls_2_vector_in_out<PointAD, VectorAD>(ftri, farc,
                                                                  v,
                                                                  m_radius);
        ps_footer(std::cerr);

        ps_header(std::cerr);
        m_weights = union_balls_2_vector_in_out<PointAD, VectorAD>(ftriw, farcw,
                                                                   v,
                                                                   m_radius);
        ps_footer(std::cerr);

        FTAD val = 0;
        for (int i = 0; i < m_values.rows(); ++i) {
            val += m_values(i);
        }

        return val;
    }

    Eigen::VectorXd grad () const {
        Eigen::VectorXd g = Eigen::VectorXd::Zero(2 * m_values.rows());

        for (int i = 0; i < m_values.rows(); ++i) {
            if (m_values(i).derivatives().rows() != 0) {
                g += m_values(i).derivatives();
            }
        }

        // Coefficent wise division
        Eigen::VectorXd w = Eigen::VectorXd::Zero(2 * m_values.rows());
        for (int i = 0; i < m_weights.rows(); ++i) {
            w[2 * i] = m_weights[i].value();
            w[2 * i + 1] = m_weights[i].value();
        }
        g = g.cwiseQuotient(w);

        return g;
    }

    private:
        double m_radius;
        VectorAD m_values;
        VectorAD m_weights;

        FTri ftri;
        FArc farc;

        FTriW ftriw;
        FArcW farcw;
};

#endif

