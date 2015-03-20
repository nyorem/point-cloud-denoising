#ifndef _UNION_BALLS_2_UTILS_HPP_
#define _UNION_BALLS_2_UTILS_HPP_

// Test if p and q are in a counter clockwise order wrt ref.
template <typename Point>
bool in_counter_clockwise (Point const& p, Point const& q,
                           Point const& ref) {
    return CGAL::left_turn(ref, p, q);
}

// Intersection between a segment and a sphere.
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
        FT ti = (-b + pm[i] * sqrt(delta)) / (2 * a);
        Point pi = s + ti * v;
        if (ti >= 0 && ti <= 1) {
            *out++ = pi;
            result = true;
        }
    }

    return result;
}

// Positive floating modulus for FT.
template <typename FT>
FT fmodpos_ft (FT x, double N) {
    while (x > N)
        x -= N;

    while (x < 0)
        x += N;

    return x;
}

// Perimeter of an angular sector defined by the vectors op and oq.
template <typename FT, typename Point, typename Vector>
FT angular_sector_perimeter (Point P, Vector op, Vector oq,
                             double radius) {
    if (op == Vector(0, 0) || oq == Vector(0, 0)) {
        return 0;
    }

    FT theta1 = fmodpos_ft(atan2(op.y(), op.x()), 2 * M_PI),
       theta2 = fmodpos_ft(atan2(oq.y(), oq.x()), 2 * M_PI);
    FT angle = fmodpos_ft(theta2 - theta1, 2 * M_PI);


    return radius * angle;
}

// Area of an angular sector defined by the vectors op and oq.
template <typename FT, typename Point, typename Vector>
FT angular_sector_area (Point P, Vector op, Vector oq,
                        double radius) {
    if (op == Vector(0, 0) || oq == Vector(0, 0)) {
        return 0;
    }

    FT theta1 = fmodpos_ft(atan2(op.y(), op.x()), 2 * M_PI),
       theta2 = fmodpos_ft(atan2(oq.y(), oq.x()), 2 * M_PI);
    FT angle = fmodpos_ft(theta2 - theta1, 2 * M_PI);


    return radius * radius * angle / 2;
}

// EXAMPLES OF FUNCTION OBJECTS

// The base class for any accumulator.
template <typename FT>
class Accumulator {
    public:
        Accumulator () : value(0) {}

        // Default operation when traversing a triangle
        template <typename Point>
        void operator() (Point const& p, Point const& q, Point const& r) {
        }

        void reset () {
            value = 0;
        }

        FT getValue () const {
            return value;
        }

    protected:
        FT value;
};

// Function object which accumlates area of triangles.
template <typename FT>
class TriangleAreaAccumulator : public Accumulator<FT> {
    public:
        using Accumulator<FT>::value;

        TriangleAreaAccumulator () : Accumulator<FT>() {}

        template <typename Point>
        void operator() (Point const& p, Point const& q, Point const& r) {
            value += CGAL::area(p, q, r);
        }
};

// Function object which accumlates perimeters of angular sectors.
template <typename FT>
class AngularSectorPerimeterAccumulator : public Accumulator<FT> {
    public:
        using Accumulator<FT>::value;

        AngularSectorPerimeterAccumulator () : Accumulator<FT>() {}

        // A full circle
        template <typename Point>
        void operator() (Point p, double radius) {
            value += 2 * M_PI * radius;
        }

        template <typename Point, typename Vector>
        void operator() (Point const& p,
                         Vector const& op, Vector const& oq,
                         double radius) {
            value += angular_sector_perimeter<FT>(p, op, oq, radius);
        }
};

// Function object which accumlates areas of angular sectors.
template <typename FT>
class AngularSectorAreaAccumulator : public Accumulator<FT> {
    public:
        using Accumulator<FT>::value;

        AngularSectorAreaAccumulator () : Accumulator<FT>() {}

        // A full circle
        template <typename Point>
        void operator() (Point p, double radius) {
            value += M_PI * radius * radius;
        }

        template <typename Point, typename Vector>
        void operator() (Point const& p,
                         Vector const& op, Vector const& oq,
                         double radius) {
            value += angular_sector_area<FT>(p, op, oq, radius);
        }
};

#endif

