#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Bbox_2.h>

#include <iostream>
#include <fstream>
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Point_2<Kernel> Point;

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

// Positive floating modulus
double fmodpos (double x, double N) {
    return std::fmod(std::fmod(x, N) + N, N);
}

// Area of an angular sector defined by the vectors op and oq
template <typename Vector>
double angular_sector_area (Vector op, Vector oq,
                            double radius) {
    if (op == Vector(0, 0) || oq == Vector(0, 0)) {
        return 0;
    }

    double theta1 = fmodpos(std::atan2(op.y(), op.x()), 2 * M_PI),
           theta2 = fmodpos(std::atan2(oq.y(), oq.x()), 2 * M_PI);
    double angle = fmodpos(theta2 - theta1, 2 * M_PI);

    return radius * radius * angle / 2;
}

// Perimeter of an angular sector defined by the vectors op and oq
template <typename Vector>
double perimeter_sector_area (Vector op, Vector oq,
                              double radius) {
    double theta1 = fmodpos(std::atan2(op.y(), op.x()), 2 * M_PI),
           theta2 = fmodpos(std::atan2(oq.y(), oq.x()), 2 * M_PI);
    double angle = fmodpos(theta2 - theta1, 2 * M_PI);

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
    typedef typename Kernel::Vector_2 Vector;
    typedef typename CGAL::Delaunay_triangulation_2<Kernel> DT;

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

    double vol = 0;

    // Decomposition of the intersection of a Voronoi cell and a ball
    // It is made of triangles and agular sectors
    for (typename DT::Finite_vertices_iterator vit = dt.finite_vertices_begin();
         vit != dt.finite_vertices_end();
         ++vit) {
        Point P = vit->point();
        if (P == bl || P == br || P == tl || P == tr) {
            continue;
        }

        // Compute the boundary of the Voronoi cell of P
        typename DT::Face_circulator fc = dt.incident_faces(vit), done(fc);
        std::vector<Point> adjacent_voronoi_vertices;
        std::cout << "Voronoi vertices of: " << P << std::endl;
        do {
            std::cout << dt.dual(fc) << std::endl;
            adjacent_voronoi_vertices.push_back(dt.dual(fc));
        } while (++fc != done);

        // Test if the first Voronoi vertex is inside
        Point p0 = adjacent_voronoi_vertices[0];
        bool isInside = (p0 - P).squared_length() <= radius * radius;
        Point lastOnBoundary(P); // Last point seen on the boundary
        bool allOutside = true;
        bool notClosed = false;

        const int N = adjacent_voronoi_vertices.size();

        // Voronoi edges
        std::vector<Segment> voronoi_edges;
        for (int i = 0; i < N; ++i) {
            voronoi_edges.push_back(Segment(adjacent_voronoi_vertices[i],
                                            adjacent_voronoi_vertices[(i + 1) % N]));
        }

        // Sort the edges by the number of intersections
        struct CompareInter {
            CompareInter(Point P, double radius) : m_center(P), m_radius(radius) {
            }

            bool operator() (Segment s1, Segment s2) {
                std::vector<Point> inter1;
                segment_sphere_intersect(m_center, m_radius, s1, std::back_inserter(inter1));

                std::vector<Point> inter2;
                segment_sphere_intersect(m_center, m_radius, s2, std::back_inserter(inter2));

                return inter1.size() > inter2.size();
            }

            private:
                Point m_center;
                double m_radius;
        } compareInter(P, radius);
        std::sort(voronoi_edges.begin(), voronoi_edges.end(), compareInter);

        // For each Voronoi vertex
        for (int i = 0; i < N; ++i) {
            Segment edge = voronoi_edges[i];
            Point p = edge.source();
            Point next = edge.target();

            // Intersection points between the Voronoi edge [p, next] and the ball
            std::cout << "Current edge: " << edge << std::endl;
            std::vector<Point> inter;
            segment_sphere_intersect(P, radius, edge, std::back_inserter(inter));
            std::cout << "Size intersection: " << inter.size() << std::endl;

            // Decompose the intersection accordingly
            if (inter.size() == 1) {
                allOutside = false;
                notClosed = false;
                if (isInside) {
                    std::cout << "Triangle: " << P << ", " << p << ", " << inter[0] << std::endl;
                    vol += std::fabs(CGAL::area(P, p, inter[0]));
                    std::cout << vol << std::endl;
                } else {
                    vol += angular_sector_area(P - lastOnBoundary, P - inter[0], radius);
                    std::cout << "Triangle: " << P << ", " << inter[0] << ", " << next << std::endl;
                    vol += std::fabs(CGAL::area(P, inter[0], next));
                    std::cout << vol << std::endl;
                }
                lastOnBoundary = inter[0];
            } else if (inter.size() == 2) {
                allOutside = false;
                notClosed = false;
                if ((inter[0] - P).squared_length() < (inter[1] - P).squared_length()) {
                    std::cout << "Sector: " << P << ", " << lastOnBoundary << ", " << inter[0]<< std::endl;
                    vol += angular_sector_area(lastOnBoundary - P, inter[0] - P, radius);
                    std::cout << vol << std::endl;
                    lastOnBoundary = inter[1];
                } else {
                    std::cout << "Sector: " << P << ", " << lastOnBoundary << ", " << inter[1]<< std::endl;
                    vol += angular_sector_area(lastOnBoundary - P, inter[1] - P, radius);
                    std::cout << vol << std::endl;
                    lastOnBoundary = inter[0];
                }
                std::cout << "Triangle: " << P << ", " << inter[0] << ", " << inter[1] << std::endl;
                vol += std::fabs(CGAL::area(P, inter[0], inter[1]));
                std::cout << vol << std::endl;
            } else {
                notClosed = true;
                bool isInsidePlus1 = (next - P).squared_length() <= radius * radius;
                if (isInside && isInsidePlus1) {
                    std::cout << "Triangle: " << P << ", " << p << ", " << next << std::endl;
                    vol += std::fabs(CGAL::area(P, p, next));
                    std::cout << vol << std::endl;
                }
            }

            // Update isInside boolean
            std::cout << "isInside = " << (isInside ? "true" : "false") << std::endl;
            if (inter.size() == 1) {
                isInside = !isInside;
            }

            std::cout << "lastOnBoundary = " << lastOnBoundary << std::endl;
        }

        // The boundary of the Voronoi cell is entirely on the outside of the ball
        std::cout << "allOutside " << allOutside << std::endl;
        if (allOutside) {
            vol += M_PI * radius * radius;
            continue;
        }

        // Close the Voronoi cell
        std::cout << "notClosed " << notClosed << std::endl;
        if (notClosed) {
            Segment firstEdge = voronoi_edges[0];
            std::vector<Point> inter;
            segment_sphere_intersect(P, radius, firstEdge, std::back_inserter(inter));
            std::cout << inter.size() << std::endl;
            if (inter.size() == 2) {
                if ((inter[0] - P).squared_length() < (inter[1] - P).squared_length()) {
                    std::cout << "Sector: " << P << ", " << lastOnBoundary << ", " << inter[0]<< std::endl;
                    vol += angular_sector_area(lastOnBoundary - P, inter[0] - P, radius);
                    std::cout << vol << std::endl;
                } else {
                    std::cout << "Sector: " << P << ", " << lastOnBoundary << ", " << inter[1]<< std::endl;
                    vol += angular_sector_area(lastOnBoundary - P, inter[1] - P, radius);
                    std::cout << vol << std::endl;
                }
            }
        }

        std::cout << std::endl;
    }

    return vol;
}

int main (int argc, const char *argv[]) {
    std::vector<Point> points;
    points.push_back(Point(0, 0));
    points.push_back(Point(2, 0));
    points.push_back(Point(1, 1));
    std::cout << volume_union_balls(points.begin(), points.end(), 1) << std::endl;

    return 0;
}

