void construct_dual (PlaneIterator pbegin,
                     PlaneIterator pbeyond,
                     Polyhedron& dual) {
    std::vector<Point_3> dual_points;
    std::map<Point_3, Plane_3> point_plane_map;

    for (PlaneIterator pit = pbegin; pit != pbeyond; ++pit) {
        Point_3 dp = CGAL::ORIGIN +
            pit->orthogonal_vector() / (-pit->d());
        dual_points.push_back(dp);
        point_plane_map[dp] = *pit;
    }
    CGAL::convex_hull_3(dual_points.begin(), dual_points.end(), dual);
    for (Vertex_iterator vit = dual.vertices_begin();
         vit != dual.vertices_end();
         ++vit) {
        vit->plane = point_plane_map[vit->point()];
    }
}

void primal_from_dual (Polyhedron const& dual,
                       Polyhedron &primal) {
    Builder B;
    B.begin_surface(m_dual.size_of_facets(),
                    m_dual.size_of_vertices(),
                    m_dual.size_of_halfedges());

    std::map<Facet_handle, size_t> primal_vertices;

    size_t n = 0;
    for (Facet_iterator fit = dual.facets_begin();
         fit != dual.facets_end();
         ++fit, ++n) {
        Halfedge_handle h = fit->halfedge();

        Plane_3 p1 = h->vertex()->plane,
                p2 = h->vertex()->next()->plane,
                p3 = h->vertex()->next()->next()->plane;

        Point_3 p = CGAL::intersection(p1, p2, p3);

        B.add_vertex(p);
        primal_vertices[fit] = n;
    }

    for (Vertex_iterator vit = dual.vertices_begin();
         vit != dual.vertices_end();
         ++vit) {
        B.begin_facet();
        Halfedge_around_vertex_circulator h0 = vit->vertex_begin(),
                                          hf = h0;

        B.begin_facet();
        do {
            B.add_vertex_to_facet(primal_vertices[hf->facet()]);
        } while (++hf != h0);
        B.end_facet();
    }

    B.end_surface();
    primal.delegate(B);
}

