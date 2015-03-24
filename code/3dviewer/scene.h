#ifndef SCENE_H
#define SCENE_H

#include <QtOpenGL/qgl.h>
#include <QString>
#include <list>

// CGAL
#include <CGAL/basic.h>
#include <CGAL/Simple_cartesian.h>
#include "polyhedron.h"
#include "pointcloud.h"
#include <CGAL/IO/Polyhedron_iostream.h>

#include "types.h"

class Scene {
    private:
        // Polyhedral surface
        Polyhedron m_polyhedron;

        // Point cloud
        Point_cloud m_pointcloud;

        // rendering options
        bool m_smooth;
        bool m_view_edges;
        bool m_view_facets;
        bool m_view_pointcloud;

    public:
        // life cycle
        Scene ();

        virtual ~Scene ();

        // file menu
        int open (QString filename);

        // algorithms menu
        void copy ();

        // toggle rendering options
        void toggle_view_smooth ()  { m_smooth = !m_smooth; }
        void toggle_view_facets ()  { m_view_facets = !m_view_facets; }
        void toggle_view_edges ()   { m_view_edges = !m_view_edges; }
        void toggle_view_pointcloud ()   { m_view_pointcloud = !m_view_pointcloud; }

        // disable rendering options
        bool is_visible_edges () const { return m_view_edges; }
        bool is_visible_facets () const { return m_view_facets; }
        bool is_visible_pointcloud () const { return m_view_pointcloud; }

        // clear scene options
        void clear_pointcloud () { m_pointcloud.clear(); }
        void clear_polyhedron () { m_polyhedron.clear(); }

        // rendering
        void render ();
        void render_edges (); // polyhedron edges
        void render_facets (); // polyhedron facets
        void render_point_cloud (); // point cloud
};

#endif // SCENE_H
