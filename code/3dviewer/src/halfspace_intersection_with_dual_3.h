#ifndef _HALFSPACE_INTERSECTION_WITH_DUAL_3_H_
#define _HALFSPACE_INTERSECTION_WITH_DUAL_3_H_

#include "tag.h"
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/intersections.h>

namespace CGAL {
    // From a list of tagged planes with AD, construct the dual polyhedron
    // using Epick.
    // The tag of a vertex is the tag of the corresponding dual plane.
    template < typename PlaneIterator,
               typename Point,
               typename Polyhedron
             >
    void construct_dual (PlaneIterator pbegin,
                         PlaneIterator pbeyond,
                         Point const& origin,
                         Polyhedron &dual) {
        typedef typename std::iterator_traits<PlaneIterator>::value_type Plane_3_ad;
        typedef typename CGAL::Kernel_traits<Plane_3_ad>::Kernel AD_kernel;
        typedef Point Point_3_ad;

        typedef CGAL::Exact_predicates_inexact_constructions_kernel Epick_kernel;
        typedef Epick_kernel::Plane_3 Plane_3_epick;
        typedef Epick_kernel::Point_3 Point_3_epick;
        typedef typename Polyhedron::Vertex_iterator Vertex_iterator;

        Point_3_epick oe(origin.x().value(),
                         origin.y().value(),
                         origin.z().value());

        // Correspondance between a dual point and a plane
        std::map<Point_3_epick, Plane_3_ad> point_plane_map;

        // Compute dual points
        std::vector<Point_3_epick> dual_points;
        for (PlaneIterator pit = pbegin;
             pit != pbeyond;
             ++pit) {
            Plane_3_epick pe(pit->a().value(),
                             pit->b().value(),
                             pit->c().value(),
                             pit->d().value());

            Plane_3_epick translated_pe(pe.a(),
                                        pe.b(),
                                        pe.c(),
                                        pe.d() + oe.x() * pe.a() + oe.y() * pe.b() + oe.z() * pe.c());

            Point_3_epick dp(CGAL::ORIGIN + translated_pe.orthogonal_vector() / (-translated_pe.d()));
            dual_points.push_back(dp);

            point_plane_map[dp] = *pit;
        }

        // Compute dual polyhedron
        CGAL::convex_hull_3(dual_points.begin(), dual_points.end(), dual);

        // Tag vertices
        for (Vertex_iterator vit = dual.vertices_begin();
             vit != dual.vertices_end();
             ++vit) {
            vit->tag = point_plane_map[vit->point()].tag;
            vit->plane = point_plane_map[vit->point()];
        }
    }

    // From a dual polyhedron (Epick), construct the primal one (AD).
    // Each facet will be tagged by the corresponding plane's tag.
    template < typename Point,
               typename Polyhedron_dual,
               typename Polyhedron_primal
             >
    class Build_primal : public CGAL::Modifier_base<typename Polyhedron_primal::HalfedgeDS> {
        private:
            typedef typename Polyhedron_primal::HalfedgeDS HDS;

            typedef typename Polyhedron_dual::Point_3 Point_3_epick;
            typedef typename CGAL::Kernel_traits<Point_3_epick>::Kernel Epick_Kernel;
            typedef typename Epick_Kernel::Plane_3 Plane_3_epick;

            Polyhedron_dual const& m_dual;
            typedef Point Point_3_ad;
            Point_3_ad const& m_origin;

            typedef typename Polyhedron_primal::Traits::Kernel AD_Kernel;
            typedef Plane_tag<AD_Kernel> Plane_3_ad;
            typedef typename AD_Kernel::FT FT_ad;

        public:
            Build_primal (Polyhedron_dual const& dual,
                          Point const& origin) : m_dual(dual),
                                                 m_origin(origin) {
            }

            void operator() (HDS &hds) {
                // Typedefs for dual
                typedef typename Polyhedron_dual::Facet Facet;
                typedef typename Polyhedron_dual::Facet_const_handle Facet_const_handle;
                typedef typename Polyhedron_dual::Facet_const_iterator Facet_const_iterator;
                typedef typename Polyhedron_dual::Vertex_const_iterator Vertex_const_iterator;

                // Typedefs for primal
                typename CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
                typedef typename Polyhedron_primal::Facet_handle Facet_handle;

                // Typedefs for intersection
                typedef typename AD_Kernel::Line_3 Line_3_ad;
                typedef boost::optional< boost::variant< Point_3_ad,
                                                         Line_3_ad,
                                                         Plane_3_ad > > result_inter;

                B.begin_surface(m_dual.size_of_facets(),
                                m_dual.size_of_vertices(),
                                m_dual.size_of_halfedges());

                // Construct primal vertices
                size_t n = 0;
                std::map<Facet_const_handle, size_t> primal_vertices;
                for (Facet_const_iterator fit = m_dual.facets_begin();
                     fit != m_dual.facets_end();
                     ++fit, ++n) {
                    typename Facet::Halfedge_const_handle h = fit->halfedge();
                    Plane_3_ad p1 = h->vertex()->plane,
                               p2 = h->next()->vertex()->plane,
                               p3 = h->next()->next()->vertex()->plane;

                    FT_ad dp1 = p1.d() + m_origin.x() * p1.a()
                        + m_origin.y() * p1.b() + m_origin.z() * p1.c();
                    FT_ad dp2 = p2.d() + m_origin.x() * p2.a()
                        + m_origin.y() * p2.b() + m_origin.z() * p2.c();
                    FT_ad dp3 = p3.d() + m_origin.x() * p3.a()
                        + m_origin.y() * p3.b() + m_origin.z() * p3.c();

                    Plane_3_ad pp1(p1.a(), p1.b(), p1.c(), dp1);
                    Plane_3_ad pp2(p2.a(), p2.b(), p2.c(), dp2);
                    Plane_3_ad pp3(p3.a(), p3.b(), p3.c(), dp3);

                    // TODO: remove auto keyword
                    auto result = CGAL::intersection(pp1, pp2, pp3);
                    const Point_3_ad* pp = boost::get<Point_3_ad>(& *result);

                    Point_3_ad ppp = m_origin + (*pp - CGAL::ORIGIN);

                    B.add_vertex(ppp);
                    primal_vertices[fit] = n;
                }

                // Construct facets
                for (Vertex_const_iterator vit = m_dual.vertices_begin();
                     vit != m_dual.vertices_end();
                     ++vit) {
                    typename Polyhedron_dual::Halfedge_around_vertex_const_circulator
                        h0 = vit->vertex_begin(), hf = h0;

                    Facet_handle handle = B.begin_facet();
                    do {
                        B.add_vertex_to_facet(primal_vertices[hf->facet()]);
                        // TODO: tag correctly the facet
                        handle->tag= vit->tag;
                    } while (++hf != h0);
                    B.end_facet();
                }

                B.end_surface();
            }
    };

    // Compute the intersection of halfspaces defined by AD planes.
    // The dual is computed using Epick.
    // The primal is constructed using AD.
    template < typename PlaneIterator,
               typename Point,
               typename Polyhedron
             >
     void halfspace_intersection_with_dual_3 (PlaneIterator pbegin,
                                              PlaneIterator pbeyond,
                                              Point const& origin,
                                              Polyhedron &P) {
        typedef Polyhedron Polyhedron_3_ad;
        typedef Point Point_3_ad;
        typedef typename CGAL::Kernel_traits<Point_3_ad>::Kernel AD_Kernel;

        typedef CGAL::Exact_predicates_inexact_constructions_kernel Epick_Kernel;
        typedef CGAL::Polyhedron_3<Epick_Kernel, Items_tag<AD_Kernel> > Polyhedron_3_epick;

        Polyhedron_3_epick dual;
        CGAL::construct_dual(pbegin, pbeyond, origin, dual);

        typedef CGAL::Build_primal<Point_3_ad, Polyhedron_3_epick, Polyhedron_3_ad> Builder_primal;
        Builder_primal builder(dual, origin);
        P.delegate(builder);
     }
} // namesapce CGAL

#endif

