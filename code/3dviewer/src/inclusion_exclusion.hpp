#ifndef _INCLUSION_EXCLUSION_HPP_
#define _INCLUSION_EXCLUSION_HPP_

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Fixed_alpha_shape_3.h>
#include <CGAL/Fixed_alpha_shape_vertex_base_3.h>
#include <CGAL/Fixed_alpha_shape_cell_base_3.h>

#include "halfspace_intersection_with_constructions_3.h"
#include "tag.h"

#include <iterator>

// Convert a standard container to a container
// of AD.
template < typename InputIterator,
           typename OutputIterator
         >
void container_to_ad (InputIterator begin,
                      InputIterator beyond,
                      OutputIterator out) {
    typedef typename std::iterator_traits<InputIterator>::value_type Point;

    for (InputIterator it = begin;
         it != beyond;
         ++it) {
        Point p = *it;
        *out++ = Point_ad(p.x(), p.y(), p.z());
    }
}

// Set the tag of all the faces of a polyhedron to true.
template < typename Polyhedron >
void mark_faces_true (Polyhedron &P) {
    typedef typename Polyhedron::Facet_iterator Facet_iterator;

    for (Facet_iterator fit = P.facets_begin();
         fit != P.facets_end();
         ++fit) {
        fit->tag = true;
    }
}

// Compute an approximation of the area of the boundary of the Minkowski
// sum of a point cloud with a convex polyhedron using an inclusion-exclusion
// formula.
template < typename FT,
           typename PointIterator,
           typename VectorIterator
         >
FT area_boundary_minkowski_sum_pointcloud_convex_polyhedron (PointIterator pbegin,
                                                             PointIterator pbeyond,
                                                             VectorIterator vbegin,
                                                             VectorIterator vbeyond,
                                                             double radius) {
    typedef typename std::iterator_traits<PointIterator>::value_type Point_3;
    typedef typename CGAL::Kernel_traits<Point_3>::Kernel Kernel;
    typedef typename Kernel::Plane_3 Plane_3;
    typedef typename Kernel::Vector_3 Vector_3;
    typedef typename CGAL::Polyhedron_3<Kernel, Items_tag<Kernel> > Polyhedron_face_tag;

    typedef CGAL::Fixed_alpha_shape_vertex_base_3<Kernel> Vb;
    typedef CGAL::Fixed_alpha_shape_cell_base_3<Kernel> Fb;
    typedef CGAL::Triangulation_data_structure_3<Vb, Fb> Tds;
    typedef CGAL::Delaunay_triangulation_3<Kernel, Tds> Triangulation_3;
    typedef CGAL::Fixed_alpha_shape_3<Triangulation_3> Alpha_shape_3;
    typedef typename Alpha_shape_3::Vertex_handle Vertex_handle;
    typedef typename Alpha_shape_3::Edge Edge;
    typedef typename Alpha_shape_3::Facet Facet;
    typedef typename Alpha_shape_3::Cell_handle Cell_handle;

    // To each point we associate the translated polyhedron (its list of faces)
    std::map<Point_3, std::list<Plane_3> > planes_map;
    for (PointIterator pit = pbegin;
         pit != pbeyond;
         ++pit) {
        std::list<Plane_3> poly;
        for (VectorIterator vit = vbegin;
             vit != vbeyond;
             ++vit) {
            Vector_3 vv = *vit,
                     pv = *pit - CGAL::ORIGIN;
            Plane_3 p(vv.x(), vv.y(), vv.z(),
                      -(pv * vv + radius));
            poly.push_back(p);
        }
        planes_map[*pit] = poly;
    }

    // Alpha-complex
    Alpha_shape_3 as(pbegin, pbeyond, radius);

    // Area of the boundary of a polyhedron
    Area_boundary_polyhedron_3<FT> ap;

    FT area = 0;
    // Vertices
    std::list<Vertex_handle> vertices;
    as.get_alpha_shape_vertices(std::back_inserter(vertices),
                                Alpha_shape_3::INTERIOR);
    as.get_alpha_shape_vertices(std::back_inserter(vertices),
                                Alpha_shape_3::REGULAR);
    as.get_alpha_shape_vertices(std::back_inserter(vertices),
                                Alpha_shape_3::SINGULAR);
    for (typename std::list<Vertex_handle>::const_iterator vit = vertices.begin();
         vit != vertices.end();
         ++vit) {
        Point_3 p = as.point(*vit);
        std::list<Plane_3> planes = planes_map[p];
        Polyhedron_face_tag P;
        CGAL::halfspace_intersection_with_constructions_3(planes.begin(),
                                                          planes.end(),
                                                          P,
                                                          p);
        mark_faces_true(P);
        ap(P);
        area += ap.getValue();
    }

    // Edges
    std::list<Edge> edges;
    as.get_alpha_shape_edges(std::back_inserter(edges),
                             Alpha_shape_3::INTERIOR);
    as.get_alpha_shape_edges(std::back_inserter(edges),
                             Alpha_shape_3::REGULAR);
    as.get_alpha_shape_edges(std::back_inserter(edges),
                             Alpha_shape_3::SINGULAR);
    for (typename std::list<Edge>::const_iterator eit = edges.begin();
         eit != edges.end();
         ++eit) {
        Point_3 p = as.segment(*eit).source(),
                q = as.segment(*eit).target();
        std::list<Plane_3> planes = planes_map[p],
            planesq = planes_map[q];
        planes.insert(planes.end(), planesq.begin(), planesq.end());
        Polyhedron_face_tag P;
        // TODO: point inside the intersection?
        CGAL::halfspace_intersection_with_constructions_3(planes.begin(),
                                                          planes.end(),
                                                          P);
        mark_faces_true(P);
        ap(P);
        area -= ap.getValue();
    }

    // Triangles
    std::list<Facet> facets;
    as.get_alpha_shape_facets(std::back_inserter(facets),
                              Alpha_shape_3::INTERIOR);
    as.get_alpha_shape_facets(std::back_inserter(facets),
                              Alpha_shape_3::REGULAR);
    as.get_alpha_shape_facets(std::back_inserter(facets),
                              Alpha_shape_3::SINGULAR);
    for (typename std::list<Facet>::const_iterator fit = facets.begin();
         fit != facets.end();
         ++fit) {
        Point_3 p = as.triangle(*fit).vertex(0),
                q = as.triangle(*fit).vertex(1),
                r = as.triangle(*fit).vertex(2);
        std::list<Plane_3> planes = planes_map[p],
            planesq = planes_map[q],
            planesr = planes_map[r];
        planes.insert(planes.end(), planesq.begin(), planesq.end());
        planes.insert(planes.end(), planesr.begin(), planesr.end());
        Polyhedron_face_tag P;
        // TODO: point inside the intersection?
        CGAL::halfspace_intersection_with_constructions_3(planes.begin(),
                                                          planes.end(),
                                                          P);
        mark_faces_true(P);
        ap(P);
        area += ap.getValue();
    }

    // Tetrahedra
    std::list<Cell_handle> cells;
    as.get_alpha_shape_cells(std::back_inserter(cells),
                             Alpha_shape_3::INTERIOR);
    as.get_alpha_shape_cells(std::back_inserter(cells),
                             Alpha_shape_3::REGULAR);
    as.get_alpha_shape_cells(std::back_inserter(cells),
                             Alpha_shape_3::SINGULAR);
    for (typename std::list<Cell_handle>::const_iterator cit = cells.begin();
         cit != cells.end();
         ++cit) {
        Point_3 p = as.tetrahedron(*cit).vertex(0),
                q = as.tetrahedron(*cit).vertex(1),
                r = as.tetrahedron(*cit).vertex(2),
                s = as.tetrahedron(*cit).vertex(3);
        std::list<Plane_3> planes = planes_map[p],
            planesq = planes_map[q],
            planesr = planes_map[r],
            planess = planes_map[s];
        planes.insert(planes.end(), planesq.begin(), planesq.end());
        planes.insert(planes.end(), planesr.begin(), planesr.end());
        planes.insert(planes.end(), planess.begin(), planess.end());
        Polyhedron_face_tag P;
        // TODO: point inside the intersection?
        CGAL::halfspace_intersection_with_constructions_3(planes.begin(),
                                                          planes.end(),
                                                          P);
        mark_faces_true(P);
        ap(P);
        area -= ap.getValue();
    }

    return area;
}

#endif

