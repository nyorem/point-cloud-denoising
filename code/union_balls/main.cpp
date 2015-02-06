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
    Vector v = seg.target() - seg.source(),
           e = s - o;

    FT a = v.squared_length(),
       b = 2 * v * e,
       c = e.squared_length() - r * r;

    FT delta = b * b - 4 * a * c;

    if (delta < 0) {
        return false;
    }

    bool result = false;
    const double pm[] = {+1.0, -1.0};
    for (size_t i = 0; i < 2; ++i) {
        FT ti = (-b + pm[i] * CGAL::sqrt(delta)) / (2 * a);
        Point pi = s + ti * v;
        if (ti >= 0 && ti <= 1) {
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
    angle = std::fabs(std::fmod(angle, 2 * M_PI));
    std::cout << "angle = " << angle << std::endl;

    return radius * radius * angle / 2;
}

// A triplet
template <typename T>
struct Triplet {
    Triplet (T p, T q, T r) : p(p), q(q), r(r) {}

    T p;
    T q;
    T r;
};

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
    typedef typename Kernel::Vector_2 Vector;
    typedef typename CGAL::Delaunay_triangulation_2<Kernel> DT;
    typedef typename DT::Face_circulator Face_circulator;
    typedef Triplet<Point> Point_triplet;

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

    // Decomposition of the intersection of a Voronoi cell and a ball
    // It is made of triangles and agular sectors
    std::map<Point, std::vector<Point_triplet> > triangles_map;
    std::map<Point, std::vector<Point_triplet> > angular_sectors_map;

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
        std::cout << "Voronoi vertices of: " << P << std::endl;
        do {
            std::cout << dt.dual(fc) << std::endl;
            adjacent_voronoi_vertices.push_back(dt.dual(fc));
        } while (++fc != done);

        // Test if the first Voronoi vertex is inside
        Point p0 = adjacent_voronoi_vertices[0];
        bool isInside = (p0 - P).squared_length() <= radius * radius;

        std::vector<Point_triplet> triangles;
        std::vector<Point_triplet> angular_sectors;

        const int N = adjacent_voronoi_vertices.size();

        // For each Voronoi vertex
        for (int i = 0; i < N; ++i) {
            Point p = adjacent_voronoi_vertices[i];
            Point next = adjacent_voronoi_vertices[(i + 1) % N];
            Segment edge(p, next);

            // Intersection points between the Voronoi edge [p, next] and the ball
            std::cout << "Current edge: " << edge << std::endl;
            std::vector<Point> inter;
            segment_sphere_intersect(P, radius, edge, std::back_inserter(inter));

            // Decompose the intersection accordingly
            if (inter.size() == 1) {
                if (isInside) {
                    triangles.push_back(Point_triplet(P, p, inter[0]));
                } else {
                    triangles.push_back(Point_triplet(P, inter[0], next));
                }
            } else if (inter.size() == 2) {
                triangles.push_back(Point_triplet(P, inter[0], inter[1]));
            } else {
                bool isInsidePlus1 = (next - P).squared_length() <= radius * radius;
                if (isInside && isInsidePlus1) {
                    triangles.push_back(Point_triplet(P, p, next));
                } else if (!isInside && !isInsidePlus1) {
                    Segment s1(P, p), s2(P, next);
                    std::cout << s1 << " / " << s2 << std::endl;
                    std::vector<Point> inter1, inter2;
                    segment_sphere_intersect(P, radius, s1, std::back_inserter(inter1));
                    segment_sphere_intersect(P, radius, s2, std::back_inserter(inter2));
                    std::cout << inter1.size() << " / " << inter2.size() << std::endl;
                    angular_sectors.push_back(Point_triplet(P, inter1[0], inter2[0]));
                }
            }

            // Update isInside boolean
            std::cout << "isInside = " << (isInside ? "true" : "false") << std::endl;
            if (inter.size() == 1) {
                isInside = !isInside;
            }
        }
        std::cout << std::endl;

        triangles_map[P] = triangles;
        angular_sectors_map[P] = angular_sectors;
    }

    // Compute the volume
    double vol = 0;
    typedef typename std::map<Point, std::vector<Point_triplet> >::iterator MapIterator;

    // Triangles
    std::cout << "triangles" << std::endl;
    std::cout << triangles_map.size() << std::endl;
    for (MapIterator mit = triangles_map.begin();
         mit != triangles_map.end();
         ++mit) {
        Point P = mit->first;
        if (P == bl || P == br ||
            P == tl || P == tr) {
            continue;
        }
        std::vector<Point_triplet>& triangles = mit->second;

        std::cout << P << std::endl;
        std::cout << triangles.size() << std::endl;
        for (int i = 0; i < triangles.size(); ++i) {
            Point_triplet& triplet = triangles[i];
            std::cout << "Triangle " << i << " : " << triplet.p << " / " << triplet.q << " / " << triplet.r << std::endl;
            vol += CGAL::area(triplet.p, triplet.q, triplet.r);
        }
        std::cout << std::endl;
    }

    // Angular sectors
    std::cout << "angular sectors" << std::endl;
    std::cout << angular_sectors_map.size() << std::endl;
    for (MapIterator mit = angular_sectors_map.begin();
         mit != angular_sectors_map.end();
         ++mit) {
        Point P = mit->first;
        if (P == bl || P == br ||
            P == tl || P == tr) {
            continue;
        }
        std::vector<Point_triplet>& angular_sectors = mit->second;

        std::cout << P << std::endl;
        std::cout << angular_sectors.size() << std::endl;
        for (int i = 0; i < angular_sectors.size(); ++i) {
            Point_triplet& sector = angular_sectors[i];
            std::cout << "Sector " << i << " : " << sector.p << " / " << sector.q << " / " << sector.r << std::endl;
            Vector pq = sector.q - sector.p,
                   pr = sector.r - sector.p;
            vol += angular_sector_area(pq, pr, radius);
        }
        std::cout << std::endl;
    }

    return vol;
}

int main (int argc, const char *argv[]) {
    std::vector<Point> points;
    points.push_back(Point(-5, 0));
    points.push_back(Point(5, 0));
    double vol = volume_union_balls(points.begin(), points.end(), 1);
    std::cout << vol << std::endl;

    Point p(0, 0), q(1, -1), r(1, 1);
    std::cout << angular_sector_area(q - p, r - p, 1) << std::endl;

    return 0;
}

