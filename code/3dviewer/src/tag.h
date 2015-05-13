#ifndef _POLYHEDRON_FACE_TAG_H_
#define _POLYHEDRON_FACE_TAG_H_

#include <CGAL/Polyhedron_3.h>

// TODO: C++11 -> inheriting constructors
// using CGAL::Plane_3<K>::Plane_3
// An extended plane with a boolean tag.
template <typename K>
struct Plane_tag : public CGAL::Plane_3<K> {
    bool tag;

    typedef K Kernel;
    typedef typename Kernel::RT RT;
    typedef typename Kernel::Point_3 Point_3;
    typedef typename Kernel::Vector_3 Vector_3;

    Plane_tag () : CGAL::Plane_3<K>(), tag(false) {
    }

    Plane_tag (RT const& a, RT const& b, RT const& c, RT const &d) :
        CGAL::Plane_3<K>(a, b, c, d), tag(false) {
    }

    Plane_tag (Point_3 const& p, Vector_3 const& v) :
        CGAL::Plane_3<K>(p, v), tag(false) {
    }

    Plane_tag (Point_3 const& p, Point_3 const& q, Point_3 const& r) :
        CGAL::Plane_3<K>(p, q, r), tag(false) {
    }
};

// An extended point with a boolean tag.
template <typename K>
struct Point_tag : public CGAL::Point_3<K> {
    bool tag;

    typedef K Kernel;
};

// Enriched face with a boolean tag.
template < typename Refs >
struct Face_with_tag :
    public CGAL::HalfedgeDS_face_base<Refs> {
        bool tag;

        Face_with_tag () : tag(false) {}
};

// Enriched vertex with a boolean tag and a dual plane (Plane_3_ad).
template < typename Refs, typename T, typename P, typename Plane >
struct Vertex_with_tag :
    public CGAL::HalfedgeDS_vertex_base<Refs, T, P> {
        bool tag;
        Plane plane;

        Vertex_with_tag () {}

        Vertex_with_tag (const P &pt) :
            CGAL::HalfedgeDS_vertex_base<Refs, T, P>(pt) {
        }
};

template <typename AD_Kernel>
struct Items_tag : public CGAL::Polyhedron_items_3 {
    template < typename Refs, typename Traits >
    struct Face_wrapper {
        typedef Face_with_tag<Refs> Face;
    };

    template < typename Refs, typename Traits >
    struct Vertex_wrapper {
        typedef typename Traits::Point_3 Point;
        typedef Plane_tag<AD_Kernel> Plane_3_ad;
        typedef Vertex_with_tag<Refs,
                                CGAL::Tag_true,
                                Point,
                                Plane_3_ad> Vertex;
    };
};

template <typename Polyhedron>
void gl_draw_edges_tag (Polyhedron const& P,
                        const float line_width,
                        const unsigned char r,
                        const unsigned char g,
                        const unsigned char b) {
    ::glLineWidth(line_width);

    ::glBegin(GL_LINES);

    ::glColor3ub(r, g, b);

    typedef typename Polyhedron::Traits::Point_3 Point;
    for (typename Polyhedron::Halfedge_const_iterator it = P.halfedges_begin();
         it != P.halfedges_end();
         ++it) {
        const Point& p1 = it->opposite()->vertex()->point();
        const Point& p2 = it->vertex()->point();

        ::glVertex3d(p1[0].value(), p1[1].value(), p1[2].value());
        ::glVertex3d(p2[0].value(), p2[1].value(), p2[2].value());
    }

    ::glEnd();
}

#endif

