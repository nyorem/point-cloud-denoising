#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Bbox_2.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <map>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Point_2<Kernel> Point;
typedef CGAL::Vector_2<Kernel> Vector;
typedef CGAL::Segment_2<Kernel> Segment;

// Intersection between a segment and a sphere
template <class Point, class Segment, class OutputIterator>
bool segment_sphere_intersect (Point o, double r,
                               Segment seg,
                               OutputIterator out) {
    typedef typename CGAL::Kernel_traits<Point>::Kernel Kernel;
    typedef typename Kernel::Vector_2 Vector;
    typedef typename Kernel::FT FT;

    Point s = seg.source();
    Vector v = seg.to_vector(),
           e = s - o;

    FT a = v.squared_length(),
       b = 2 * v * e,
       c = e.squared_length() - r * r;

    FT delta = b * b - 4 * a * c;

    bool result = false;

    const double pm[] = {+1.0, -1.0};
    for (size_t i = 0; i < 2; ++i) {
        FT ti = (-b + pm[i] * CGAL::sqrt(delta)) / (2 * a);
        Point pi = s + ti * v;
        if (seg.has_on(pi)) {
            *out++ = pi;
            result = true;
        }
    }

    return result;
}

// Compute the volume of the radius-offset of the point cloud
// InputIterator::value_type = Point
template <typename InputIterator>
double volume_union_balls (InputIterator begin,
                           InputIterator beyond,
                           double radius) {
    typedef typename std::iterator_traits<InputIterator>::value_type Point;
    typedef typename CGAL::Kernel_traits<Point>::Kernel Kernel;
    typedef typename Kernel::FT FT;
    typedef typename Kernel::Segment_2 Segment;
    typedef typename CGAL::Delaunay_triangulation_2<Kernel> DT;
    typedef typename DT::Face_circulator Face_circulator;

    // Bounding box
    CGAL::Bbox_2 b = CGAL::bbox_2(begin, beyond);
    Point bl(b.xmin() - 2 * radius, b.ymin() - 2 * radius),
          br(b.xmax() + 2 * radius, b.ymin() - 2 * radius),
          tl(b.xmin() - 2 * radius, b.ymax() + 2 * radius),
          tr(b.xmax() + 2 * radius, b.ymax() + 2 * radius);

    // Delaunay triangulation
    DT dt;
    dt.insert(bl);
    dt.insert(br);
    dt.insert(tl);
    dt.insert(tr);
    dt.insert(begin, beyond);

    // Intersection of the Voronoi cell with the balls
    std::map<Point, std::vector<Point> > intersection_points;

    for (typename DT::Finite_vertices_iterator vit = dt.finite_vertices_begin();
         vit != dt.finite_vertices_end();
         ++vit) {
        Point P = vit->point();

        // Compute the boundary of the Voronoi cell of P
        Face_circulator fc = dt.incident_faces(vit),
                        done(fc++);
        std::vector<Point> adjacent_voronoi_vertices;
        while (fc != done) {
            adjacent_voronoi_vertices.push_back(dt.dual(fc));
            fc++;
        }

        // Test if the first point is inside
        Point p = adjacent_voronoi_vertices[0];
        FT distance = (p - P).squared_length();
        bool isInside = distance <= radius * radius;

        // For each Voronoi vertex
        for (int i = 0; i < adjacent_voronoi_vertices.size(); ++i) {
            Point p = adjacent_voronoi_vertices[i];

            // Intersection points between the Voronoi edge and the ball
            Point next = adjacent_voronoi_vertices[(i + 1) % adjacent_voronoi_vertices.size()];
            Segment seg(p, next);
            std::vector<Point> inter;
            segment_sphere_intersect(P, radius, seg, std::back_inserter(inter));
            intersection_points[P] = inter;

            // Update isInside boolean
            if (intersection_points.size() == 1) {
                isInside = !isInside;
            }
        }
    }

    // TODO
    return 0;
}

int main (int argc, const char *argv[]) {
    std::vector<Point> points;
    points.push_back(Point(0, 0));
    double vol = volume_union_balls(points.begin(), points.end(), 1);

    /* std::vector<Point> points; */
    /* Segment s(Point(-1, -1), Point(1, 1)); */
    /* segment_sphere_intersect(Point(0, 0), 1, */
    /*                          s, */
    /*                          std::back_inserter(points)); */

    /* for (int i = 0; i < points.size(); ++i) { */
    /*     std::cout << points[i] << std::endl; */
    /* } */

    return 0;
}

