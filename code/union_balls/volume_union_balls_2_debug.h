#ifndef _VOLUME_UNION_BALLS_2_DEBUG_H_
#define _VOLUME_UNION_BALLS_2_DEBUG_H_

#include "volume_union_balls_2.h"

// Version of volume_ball_voronoi_cell_2 outputing
// a list of segments corresponding to the decomposition
template <typename Kernel, typename DT, typename Vertex_handle>
typename Kernel::FT volume_ball_voronoi_cell_2_debug (DT const& dt,
                                                      Vertex_handle const& v,
                                                      double radius,
                                                      std::vector<typename Kernel::Segment_2>& out) {
    typedef typename Kernel::Point_2 Point;
    typedef typename Kernel::Line_2 Line;
    typedef typename Kernel::FT FT;
    typedef typename Kernel::Segment_2 Segment;

    Point P = v->point();

    FT vol = 0;

    // Compute the boundary of the Voronoi cell of P
    typename DT::Face_circulator fc = dt.incident_faces(v), done(fc);
    std::vector<Point> adjacent_voronoi_vertices;
    std::cout << "Voronoi vertices of: " << P << std::endl;
    do {
        std::cout << dt.dual(fc) << std::endl;
        adjacent_voronoi_vertices.push_back(dt.dual(fc));
    } while (++fc != done);

    const int N = adjacent_voronoi_vertices.size();

    // Test if the first Voronoi vertex is inside
    Point p0 = adjacent_voronoi_vertices[0];
    bool isInside = (p0 - P).squared_length() <= radius * radius;
    bool allOutside = true;

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

    out.clear();

    // The boundary of the Voronoi cell is entirely outside of the ball
    std::cout << "allOutside " << allOutside << std::endl;
    if (allOutside) {
        vol = M_PI * radius * radius;
        std::cout << "vol = " << vol << std::endl;

        return vol;
    }

    // Special case: 2 points
    if (boundary.size() == 2) {
        Point p = boundary[0], pp = boundary[1];
        Line l(p, pp);

        out.push_back(Segment(P, p));
        out.push_back(Segment(P, pp));
        out.push_back(Segment(p, pp));

        if (l.oriented_side(P) == CGAL::ON_POSITIVE_SIDE) {
            vol += CGAL::area(P, p, pp);
            std::cout << "vol = " << vol << std::endl;
            vol += angular_sector_area(pp - P, p - P, radius);
            std::cout << "vol = " << vol << std::endl;
        } else {
            vol += CGAL::area(P, pp, p);
            std::cout << "vol = " << vol << std::endl;
            vol += angular_sector_area(p - P, pp - P, radius);
            std::cout << "vol = " << vol << std::endl;
        }

        return vol;
    }

    for (size_t i = 0; i < boundary.size(); ++i) {
        Point p = boundary[i],
              pp = boundary[(i + 1) % boundary.size()];

        if (interior_map[p] && interior_map[pp]) {
            // 2 interior points: triangle
            vol += CGAL::area(P, p, pp);
            out.push_back(Segment(P, p));
            out.push_back(Segment(P, pp));
            out.push_back(Segment(p, pp));
            std::cout << "vol = " << vol << std::endl;
        } else if (interior_map[p] && !interior_map[pp]) {
            // 1 interior point: triangle
            vol += CGAL::area(P, p, pp);
            out.push_back(Segment(P, p));
            out.push_back(Segment(P, pp));
            out.push_back(Segment(p, pp));
            std::cout << "vol = " << vol << std::endl;
        } else if (!interior_map[p] && interior_map[pp]) {
            // 1 interior point: triangle
            vol += CGAL::area(P, p, pp);
            out.push_back(Segment(P, p));
            out.push_back(Segment(P, pp));
            out.push_back(Segment(p, pp));
            std::cout << "vol = " << vol << std::endl;
        } else {
            // 0 interior points: 2 on the boundary
            // It depends on the corresponding edges
            Segment pedge = edge_map[p],
                    ppedge = edge_map[pp];

            if (pedge == ppedge) {
                // Same edge: triangle
                vol += CGAL::area(P, p, pp);
                out.push_back(Segment(P, p));
                out.push_back(Segment(P, pp));
                out.push_back(Segment(p, pp));
                std::cout << "vol = " << vol << std::endl;
            } else {
                // Different edges: angular sector
                vol += angular_sector_area(p - P, pp - P, radius);
                out.push_back(Segment(P, p));
                out.push_back(Segment(P, pp));
                std::cout << "vol = " << vol << std::endl;
            }
        }
    }

    std::cout << std::endl;

    return vol;
}

// Version of volume_union_balls_2 outputing
// a list of segments corresponding to the decomposition
template <typename FT, typename Segment, typename InputIterator>
FT volume_union_balls_2_debug (InputIterator begin,
                               InputIterator beyond,
                               double radius,
                               std::vector<Segment>& out) {
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

    FT vol = 0;

    out.clear();
    // Decomposition of the intersection of a Voronoi cell and a ball
    // It is made of triangles and agular sectors
    for (typename DT::Finite_vertices_iterator vit = dt.finite_vertices_begin();
         vit != dt.finite_vertices_end();
         ++vit) {
        Point P = vit->point();

        if (P == bl || P == br || P == tl || P == tr) {
            continue;
        }

        std::vector<Segment> segments;
        vol += volume_ball_voronoi_cell_2_debug<Kernel>(dt, vit, radius, segments);
        out.insert(out.end(), segments.begin(), segments.end());
    }

    return vol;
}

#endif

