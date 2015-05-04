#ifndef _MINKOWSKI_SUM_POINTCLOUD_CONVEX_POLYHEDRON_3_H_
#define _MINKOWSKI_SUM_POINTCLOUD_CONVEX_POLYHEDRON_3_H_

#include <CGAL/Delaunay_triangulation_3.h>
#include "halfspace_intersection_with_dual_3.h"
#include "halfspace_intersection_with_constructions_3.h"
#include "minkowski_sum_pointcloud_convex_polyhedron_utils_3.h"

#include <iterator>

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
    typedef Plane_tag<Kernel> Plane_3;
    typedef typename CGAL::Polyhedron_3<Kernel, Items_tag<Kernel> > Polyhedron_tag;
    typedef typename Polyhedron_tag::Facet_iterator Facet_iterator;
    typedef typename Polyhedron_tag::Facet::Halfedge_handle Halfedge_handle;

    std::list<Plane_3> boundary;

    std::list<Vertex_handle> neighbours;
    dt.adjacent_vertices(v, std::back_inserter(neighbours));

    // Voronoi faces
    for (typename std::list<Vertex_handle>::iterator it = neighbours.begin();
         it != neighbours.end();
         ++it) {
        if (! dt.is_infinite(*it)) {
            Vector_3 p = ((*it)->point() - v->point()) / 2;
            Plane_3 pp(CGAL::midpoint((*it)->point(), v->point()), p);
            pp.tag = false;
            boundary.push_back(pp);
        }
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
        p.tag = true;
        boundary.push_back(p);
        poly.push_back(p);
    }

    // Intersection
    Polyhedron_tag P;
    CGAL::halfspace_intersection_with_constructions_3(boundary.begin(),
                                                      boundary.end(),
                                                      P,
                                                      v->point());
    /* CGAL::halfspace_intersection_with_dual_3(boundary.begin(), */
    /*                                          boundary.end(), */
    /*                                          v->point(), */
    /*                                          P); */

    // TODO: use directly the tag present in the polyhedron's facets
    // Tag faces with true if they belong to the convex polyhedron
    // and false otherwise
    for (Facet_iterator fit = P.facets_begin();
         fit != P.facets_end();
         ++fit) {
        Halfedge_handle h(fit->halfedge());
        Plane_3 p(h->vertex()->point(),
                  h->next()->vertex()->point(),
                  h->next()->next()->vertex()->point());

        // Check if the facet is a polyhedron or a Voronoi facet
        // TODO: why p.opposite() instead of p?
        std::cout << "tag facet " << fit->tag << std::endl;
        if (std::find(poly.begin(), poly.end(), p.opposite()) != poly.end()) {
            std::cout << "true" << std::endl;
            fit->tag = true;
        } else {
            std::cout << "false" << std::endl;
            fit->tag = false;
        }
    }

    // Compute a value along the intersection
    acc(P);
    std::cout << std::endl;
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

    for (typename DT::Finite_vertices_iterator vit = dt.finite_vertices_begin();
         vit != dt.finite_vertices_end();
         ++vit) {
        Point_3 P = vit->point();

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

    int i = 0;
    for (typename DT::Finite_vertices_iterator vit = dt.finite_vertices_begin();
         vit != dt.finite_vertices_end();
         ++vit) {
        Point_3 P = vit->point();

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

