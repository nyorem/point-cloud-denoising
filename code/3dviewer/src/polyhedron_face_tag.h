#ifndef _POLYHEDRON_FACE_TAG_H_
#define _POLYHEDRON_FACE_TAG_H_

#include <CGAL/Polyhedron_3.h>

// Enriched face with a boolean tag.
template < typename Refs >
struct Face_with_tag : public CGAL::HalfedgeDS_face_base<Refs> {
    bool tag;
};

struct Items_face_tag : public CGAL::Polyhedron_items_3 {
    template < typename Refs, typename Traits >
    struct Face_wrapper {
        typedef Face_with_tag<Refs> Face;
    };
};

#endif

