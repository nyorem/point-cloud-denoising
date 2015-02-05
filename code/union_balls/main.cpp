#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Bbox_2.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <map>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Point_2<Kernel> Point;
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

    if (delta < 0 || a == 0) {
        return false;
    }

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

// Area of an angular sector defined by the vectors op and oq
template <typename Vector>
double angular_sector_area (Vector op, Vector oq,
                            double radius) {
    double angle = std::atan2(op.y(), op.x()) - std::atan2(oq.y(), oq.x());

    if (angle < 0) {
        angle += 2 * M_PI;
    }

    return radius * angle;
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
    DT dt(begin, beyond);
    dt.insert(bl); dt.insert(br);
    dt.insert(tl); dt.insert(tr);

    // Intersection of the Voronoi cell with the balls
    std::map<Point, std::vector<Point> > intersection_points;
    // Interior points
    std::map<Point, std::vector<Point> > interior_points;

    for (typename DT::Finite_vertices_iterator vit = dt.finite_vertices_begin();
         vit != dt.finite_vertices_end();
         ++vit) {
        Point P = vit->point();
        if (P == bl || P == br || P == tl || P == tr) {
            continue;
        }

        // Compute the boundary of the Voronoi cell of P
        Face_circulator fc = dt.incident_faces(vit), done(fc);
        std::vector<Point> adjacent_voronoi_vertices;
        std::cout << P << std::endl;
        do {
            std::cout << dt.dual(fc) << std::endl;
            adjacent_voronoi_vertices.push_back(dt.dual(fc));
        } while (++fc != done);
        std::cout << adjacent_voronoi_vertices.size() << std::endl << std::endl;

        // Test if the first point is inside
        Point p0 = adjacent_voronoi_vertices[0];
        FT distance = (p0 - P).squared_length();
        bool isInside = distance <= radius * radius;

        std::vector<Point> interior;
        std::vector<Point> intersection;

        // For each Voronoi vertex
        for (int i = 0; i < adjacent_voronoi_vertices.size(); ++i) {
            // Interior points
            Point p = adjacent_voronoi_vertices[i];
            if (isInside) {
                interior.push_back(p);
            }

            // Intersection points between the Voronoi edge [p, next] and the ball
            Point next = adjacent_voronoi_vertices[(i + 1) % adjacent_voronoi_vertices.size()];
            Segment seg(p, next);
            std::cout << seg << std::endl;
            std::vector<Point> inter;
            if (segment_sphere_intersect(P, radius, seg, std::back_inserter(inter))) {
                intersection.insert(intersection.end(), inter.begin(), inter.end());
            }

            // Update isInside boolean
            std::cout << (isInside ? "true" : "false") << std::endl;
            if (intersection_points.size() == 1) {
                isInside = !isInside;
            }
        }
        std::cout << std::endl;

        interior_points[P] = interior;
        intersection_points[P] = intersection;
    }

    // Compute the volume
    double vol = 0;
    typedef typename std::map<Point, std::vector<Point> >::iterator MapIterator;

    // Intersection points
    // TODO
    std::cout << "intersection" << std::endl;
    std::cout << intersection_points.size() << std::endl;
    for (MapIterator mit = intersection_points.begin();
         mit != intersection_points.end();
         ++mit) {
        Point P = mit->first;
        if (P == bl || P == br ||
            P == tl || P == tr) {
            continue;
        }
        std::vector<Point>& inter = mit->second;

        std::cout << P << std::endl;
        std::cout << inter.size() << std::endl;
        for (int i = 0; i < inter.size(); ++i) {
            Point& p = inter[i];
            Point& next = inter[(i + 1) % inter.size()];
            std::cout << p << ", " << next << std::endl;
            vol += angular_sector_area(p - P, next - P, radius);
        }
        std::cout << std::endl;
    }

    // Interior points
    // TODO
    std::cout << "interior" << std::endl;
    std::cout << interior_points.size() << std::endl;
    for (MapIterator mit = interior_points.begin();
         mit != interior_points.end();
         ++mit) {
        Point P = mit->first;
        if (P == bl || P == br ||
            P == tl || P == tr) {
            continue;
        }
        std::vector<Point>& inter = mit->second;

        std::cout << P << std::endl;
        std::cout << inter.size() << std::endl;
        for (int i = 0; i < static_cast<int>(inter.size()) - 1; ++i) {
            Point& p = inter[i];
            Point& next = inter[i + 1];
            std::cout << p << ", " << next << std::endl;
            vol += CGAL::area(P, p, next);
        }
        std::cout << std::endl;
    }

    return vol;
}

int main (int argc, const char *argv[]) {
    std::vector<Point> points;
    points.push_back(Point(-1, 0));
    points.push_back(Point(1, 0));
    double vol = volume_union_balls(points.begin(), points.end(), 2);

    std::cout << vol << std::endl;

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

