#ifndef _MINKOWSKI_SUM_POINTCLOUD_CONVEX_POLYHEDRON_3_H_
#define _MINKOWSKI_SUM_POINTCLOUD_CONVEX_POLYHEDRON_3_H_

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Bbox_3.h>
#include "halfspace_intersection_with_constructions_3.h"

#include <iterator>
#include "minkowski_sum_pointcloud_convex_polyhedron_utils_3.h"

// Compute a value along the intersection of a Voronoi cell and
// a convex polyhedron.
// The convex polyhedron is defined by a list of vectors such that
// P = { x | (x | v_i) <= 1 }.
// -> PointIterator::value_type = Point_3
// -> VectorIterator::value_type = Vector_3
template < typename FT,
           typename Accum,
           typename VectorIterator,
           typename DT,
           typename Vertex_handle
         >
void voronoi_cell_convex_polyhedron_3 (DT const& dt,
                                       Vertex_handle const& v,
                                       VectorIterator polybegin,
                                       VectorIterator polybeyond,
                                       Accum &acc,
                                       double radius) {
    typedef typename std::iterator_traits<VectorIterator>::value_type Vector_3;
    typedef typename CGAL::Kernel_traits<Vector_3>::Kernel Kernel;
    typedef typename Kernel::Plane_3 Plane_3;
    typedef typename CGAL::Polyhedron_3<Kernel> Polyhedron_3;

    std::list<Plane_3> boundary;

    std::list<Vertex_handle> neighbours;
    dt.adjacent_vertices(v, std::back_inserter(neighbours));

    // Voronoi faces
    for (typename std::list<Vertex_handle>::iterator it = neighbours.begin();
         it != neighbours.end();
         ++it) {
        Vector_3 p = ((*it)->point() - v->point()) / 2;
        boundary.push_back(Plane_3(CGAL::midpoint((*it)->point(), v->point()),
                                   p));
    }

    // Translated polyhedron
    std::list<Plane_3> poly;
    for (VectorIterator vit = polybegin;
         vit != polybeyond;
         ++vit) {
        Vector_3 vv = *vit,
                 pv = v->point() - CGAL::ORIGIN;
        Plane_3 p(vv.x(), vv.y(), vv.z(),
                  -(pv * vv + radius));
        boundary.push_back(p);
    }

    // Intersection
    Polyhedron_3 P;
    CGAL::halfspace_intersection_with_constructions_3(boundary.begin(),
                                                      boundary.end(),
                                                      P,
                                                      v->point());

    // Compute a value
    acc(P);
}

// Compute a value along the Minkowski sum of a point cloud
// with a convex polyhedron.
// The convex polyhedron is defined by a list of vectors such that
// P = { x | (x | v_i) <= 1 }.
// -> PointIterator::value_type = Point_3
// -> VectorIterator::value_type = Vector_3
template < typename FT,
           typename Accum,
           typename PointIterator,
           typename VectorIterator
         >
void minkowski_sum_pointcloud_convex_polyhedron_3 (PointIterator pbegin,
                                                   PointIterator pbeyond,
                                                   VectorIterator polybegin,
                                                   VectorIterator polybeyond,
                                                   Accum &acc,
                                                   double radius) {
    typedef typename std::iterator_traits<VectorIterator>::value_type Vector_3;
    typedef typename CGAL::Kernel_traits<Vector_3>::Kernel Kernel;
    typedef CGAL::Point_3<Kernel> Point_3;
    typedef typename CGAL::Delaunay_triangulation_3<Kernel> DT;
    typedef typename DT::Vertex_handle Vertex_handle;

    FT vol = 0;

    DT dt(pbegin, pbeyond);

    // Bounding box: make Voronoi cells bounded
    CGAL::Bbox_3 bb = CGAL::bbox_3(pbegin, pbeyond);
    Point_3 a(bb.xmin() - 2 * radius, bb.ymin() - 2 * radius, bb.zmin() - 2 * radius),
            b(bb.xmin() - 2 * radius, bb.ymax() + 2 * radius, bb.zmin() - 2 * radius),
            c(bb.xmax() + 2 * radius, bb.ymax() + 2 * radius, bb.zmin() - 2 * radius),
            d(bb.xmax() + 2 * radius, bb.ymin() - 2 * radius, bb.zmin() - 2 * radius),
            e(bb.xmin() - 2 * radius, bb.ymin() - 2 * radius, bb.zmax() + 2 * radius),
            f(bb.xmin() - 2 * radius, bb.ymax() + 2 * radius, bb.zmax() + 2 * radius),
            g(bb.xmax() + 2 * radius, bb.ymax() + 2 * radius, bb.zmax() + 2 * radius),
            h(bb.xmax() + 2 * radius, bb.ymin() - 2 * radius, bb.zmax() + 2 * radius);
    dt.insert(a); dt.insert(b); dt.insert(c); dt.insert(d);
    dt.insert(e); dt.insert(f); dt.insert(g); dt.insert(h);

    for (typename DT::Finite_vertices_iterator vit = dt.finite_vertices_begin();
         vit != dt.finite_vertices_end();
         ++vit) {
        Point_3 P = vit->point();

        if (P == a || P == b || P == c || P == d ||
            P == e || P == f || P == g || P == h) {
            continue;
        }

        voronoi_cell_convex_polyhedron_3<FT>(dt, Vertex_handle(vit),
                                             polybegin,
                                             polybeyond,
                                             acc,
                                             radius);
    }
}

// Same as `minkowski_sum_pointcloud_convex_polyhedron_3` but outputs an Eigen vector
// containing for each index i the value of the ith voronoi cell intersected
// with the convex polyhedron.
template < typename FT,
           typename Vector,
           typename Accum,
           typename PointIterator,
           typename VectorIterator
         >
Vector minkowski_sum_pointcloud_convex_polyhedron_vector_out_3 (PointIterator pbegin,
                                                                PointIterator pbeyond,
                                                                VectorIterator polybegin,
                                                                VectorIterator polybeyond,
                                                                Accum &acc,
                                                                double radius) {
    typedef typename std::iterator_traits<VectorIterator>::value_type Vector_3;
    typedef typename CGAL::Kernel_traits<Vector_3>::Kernel Kernel;
    typedef CGAL::Point_3<Kernel> Point_3;
    typedef typename CGAL::Delaunay_triangulation_3<Kernel> DT;
    typedef typename DT::Vertex_handle Vertex_handle;

    int N = 0;
    for (PointIterator it = pbegin; it != pbeyond; ++it) {
        ++N;
    }
    Vector val(N);

    DT dt(pbegin, pbeyond);

    // Bounding box: make Voronoi cells bounded
    CGAL::Bbox_3 bb = CGAL::bbox_3(pbegin, pbeyond);
    Point_3 a(bb.xmin() - 2 * radius, bb.ymin() - 2 * radius, bb.zmin() - 2 * radius),
            b(bb.xmin() - 2 * radius, bb.ymax() + 2 * radius, bb.zmin() - 2 * radius),
            c(bb.xmax() + 2 * radius, bb.ymax() + 2 * radius, bb.zmin() - 2 * radius),
            d(bb.xmax() + 2 * radius, bb.ymin() - 2 * radius, bb.zmin() - 2 * radius),
            e(bb.xmin() - 2 * radius, bb.ymin() - 2 * radius, bb.zmax() + 2 * radius),
            f(bb.xmin() - 2 * radius, bb.ymax() + 2 * radius, bb.zmax() + 2 * radius),
            g(bb.xmax() + 2 * radius, bb.ymax() + 2 * radius, bb.zmax() + 2 * radius),
            h(bb.xmax() + 2 * radius, bb.ymin() - 2 * radius, bb.zmax() + 2 * radius);
    dt.insert(a); dt.insert(b); dt.insert(c); dt.insert(d);
    dt.insert(e); dt.insert(f); dt.insert(g); dt.insert(h);

    int i = 0;
    for (typename DT::Finite_vertices_iterator vit = dt.finite_vertices_begin();
         vit != dt.finite_vertices_end();
         ++vit) {
        Point_3 P = vit->point();

        if (P == a || P == b || P == c || P == d ||
            P == e || P == f || P == g || P == h) {
            continue;
        }

        acc.reset();

        voronoi_cell_convex_polyhedron_3<FT>(dt, Vertex_handle(vit),
                                             polybegin,
                                             polybeyond,
                                             acc,
                                             radius);

        val[i++] = acc.getValue();
    }

    return val;
}

// Same as `minkowski_sum_pointcloud_convex_polyhedron_3` but takes a vector in argument
// instead of a STL container.
template < typename FT,
           typename Accum,
           typename Point,
           typename Vector,
           typename VectorIterator
         >
void minkowski_sum_pointcloud_convex_polyhedron_vector_in_3 (Vector points,
                                                             VectorIterator polybegin,
                                                             VectorIterator polybeyond,
                                                             Accum &acc,
                                                             double radius) {
    std::list<Point> vec;
    vector_to_container<Point>(points, std::back_inserter(vec));

    minkowski_sum_pointcloud_convex_polyhedron_3<FT>(vec.begin(), vec.end(),
                                                     polybegin, polybeyond,
                                                     acc,
                                                     radius);
}

// Same as `minkowski_sum_pointcloud_convex_polyhedron_3` but takes an Eigen vector
// as argument and outputs an Eigen vector.
template < typename FT,
           typename Point,
           typename Accum,
           typename Vector,
           typename VectorIterator
          >
Vector minkowski_sum_pointcloud_convex_polyhedron_vector_in_out_3 (Vector points,
                                                                   VectorIterator polybegin,
                                                                   VectorIterator polybeyond,
                                                                   Accum &acc,
                                                                   double radius) {
    std::list<Point> vec;
    vector_to_container<Point>(points, std::back_inserter(vec));

    return minkowski_sum_pointcloud_convex_polyhedron_vector_out_3<FT, Vector>(vec.begin(), vec.end(),
                                                                               polybegin, polybeyond,
                                                                               acc,
                                                                               radius);
}

// Using Automatic Differentiation, compute the gradient of values along
// intersections of Voronoi cells and polyhedra.
template < typename PointAD,
           typename FTAD,
           typename VectorAD,
           typename Accum
         >
struct UnionPolyhedra {
    UnionPolyhedra (double radius) : m_radius(radius), m_acc() {
    }

    template <typename PointIterator, typename VectorIterator>
    FTAD operator() (PointIterator pbegin, PointIterator pbeyond,
                     VectorIterator vbegin, VectorIterator vbeyond) {
        m_values = minkowski_sum_pointcloud_convex_polyhedron_vector_in_out_3<FTAD, PointAD>(container_to_vector<VectorAD>(pbegin, pbeyond),
                                                                                             vbegin, vbeyond,
                                                                                             m_acc,
                                                                                             m_radius);

        FTAD val = 0;
        for (int i = 0; i < m_values.rows(); ++i) {
            val += m_values(i);
        }

        return val;
    }

    Eigen::VectorXd grad () const {
        Eigen::VectorXd g = Eigen::VectorXd::Zero(3 * m_values.rows());

        for (int i = 0; i < m_values.rows(); ++i) {
            if (m_values(i).derivatives().rows() != 0) {
                g += m_values(i).derivatives();
            }
        }

        return g;
    }

    private:
        double m_radius;
        VectorAD m_values;
        Accum m_acc;
};

#endif

