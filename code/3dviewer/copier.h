#ifndef _COPIER_
#define _COPIER_

#include "polyhedron.h"
#include <CGAL/Polyhedron_incremental_builder_3.h>

template <class HDS, class Polyhedron, class Kernel>
class CModifierCopy : public CGAL::Modifier_base<HDS> {
    private:
        typedef typename HDS::Vertex          Vertex;
        typedef typename Vertex::Point        Point;
        typedef typename HDS::Face_handle     Face_handle;
        typedef typename HDS::Halfedge_handle Halfedge_handle;
        typedef typename CGAL::Polyhedron_incremental_builder_3<HDS> Builder;

        Polyhedron *m_pMesh;

    public:
        // life cycle
        CModifierCopy(Polyhedron *pMesh)
        {
            CGAL_assertion(pMesh != NULL);
            m_pMesh = pMesh;
        }
        ~CModifierCopy() {}

        // copy
        void operator() (HDS& hds) {
            Builder B(hds,true);
            B.begin_surface(3,1,6);
            add_vertices(B);
            add_facets(B);
            B.end_surface();
        }

        //***************************************************
        // add vertices
        //***************************************************
        void add_vertices (Builder &B) {
            // copy original vertices
            int index = 0;

            typename Polyhedron::Vertex_iterator pVertex;

            for(pVertex = m_pMesh->vertices_begin();
                pVertex != m_pMesh->vertices_end();
                pVertex++) {
                pVertex->tag(index);
                B.add_vertex(pVertex->point());
                index++;
            }
        }

        //***************************************************
        // add facets
        //***************************************************
        void add_facets(Builder &B) {
            // copy original facets
            typename Polyhedron::Facet_iterator pFacet;

            for(pFacet = m_pMesh->facets_begin();
                pFacet != m_pMesh->facets_end();
                pFacet++) {
                // begin facet assembly
                B.begin_facet();
                typename Polyhedron::Halfedge_around_facet_circulator h;
                h = pFacet->facet_begin();
                do {
                    int index_vertex = h->vertex()->tag();
                    B.add_vertex_to_facet(index_vertex);
                } while(++h != pFacet->facet_begin());

                B.end_facet();
            }
        }

};

template <class Polyhedron, class Kernel>
class CCopier {
    public:
        typedef typename Polyhedron::HalfedgeDS HalfedgeDS;
        CCopier() {}
        ~CCopier() {}

    public:
        void copy (Polyhedron &input,
                   Polyhedron &output) {
            // copy
            CModifierCopy<HalfedgeDS, Polyhedron, Kernel> builder(&input);
            output.delegate(builder);
        }
};

#endif // _COPIER_
