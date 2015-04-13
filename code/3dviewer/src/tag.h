#ifndef _POLYHEDRON_FACE_TAG_H_
#define _POLYHEDRON_FACE_TAG_H_

#include <CGAL/Polyhedron_3.h>

// Enriched face with a boolean tag.
template < typename Refs >
struct Face_with_tag :
    public CGAL::HalfedgeDS_face_base<Refs> {
        bool tag;

        Face_with_tag () {}
};

// Enriched vertex with a boolean tag.
template < typename Refs, typename T, typename P >
struct Vertex_with_tag :
    public CGAL::HalfedgeDS_vertex_base<Refs, T, P> {
        bool tag;

        Vertex_with_tag () {}

        Vertex_with_tag (const P &pt) :
            CGAL::HalfedgeDS_vertex_base<Refs, T, P>(pt) {
            }
};

struct Items_tag : public CGAL::Polyhedron_items_3 {
    template < typename Refs, typename Traits >
    struct Face_wrapper {
        typedef Face_with_tag<Refs> Face;
    };

    template < typename Refs, typename Traits >
    struct Vertex_wrapper {
        typedef typename Traits::Point_3 Point;
        typedef Vertex_with_tag<Refs,
                                CGAL::Tag_true,
                                Point> Vertex;
    };
};

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

#endif

