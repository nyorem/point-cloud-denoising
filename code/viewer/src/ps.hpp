#ifndef _PS_HPP_
#define _PS_HPP_

// Scale / translation parameters
static const double sx = 0.5;
static const double sy = 0.5;
static const double sr = 0.5;

static const int tx = 200;
static const int ty = 200;

void ps_header (std::ostream& os) {
    os << "%!PS" << std::endl;
}

// Draw a CGAL triangle
template <typename K>
void ps_triangle (std::ostream& os,
                  typename CGAL::Point_2<K> const& p,
                  typename CGAL::Point_2<K> const& pp,
                  typename CGAL::Point_2<K> const& ppp,
                  const unsigned int r = 0,
                  const unsigned int g = 0,
                  const unsigned int b = 0,
                  const bool filled = false) {
    os << "newpath" << std::endl;
    os << sx * p.x() + tx << " " << sy * p.y() + ty << " moveto" << std::endl;
    os << sx * pp.x() + tx << " " << sy * pp.y() + ty << " lineto" << std::endl;
    os << sx * ppp.x() + tx << " " << sy * ppp.y() + ty << " lineto" << std::endl;
    os << sx * p.x() + tx << " " << sy * p.y() + ty << " lineto" << std::endl;
    os << "closepath" << std::endl;
    os << r << " " << g << " " << b << " setrgbcolor" << std::endl;
    os << (filled ? "filled" : "stroke") << std::endl;
}

// Draw an angular sector
template <typename K>
void ps_arc (std::ostream& os,
             typename CGAL::Point_2<K> const& center,
             double radius,
             double startAngle, // in radians
             double endAngle, // in radians
             const bool closed = true,
             const unsigned int r = 0,
             const unsigned int g = 0,
             const unsigned int b = 0,
             const bool filled = false) {

    double startAngleD = startAngle * 180 / M_PI;
    double endAngleD = endAngle * 180 / M_PI;

    os << r << " " << g << " " << b << " setrgbcolor" << std::endl;
    os << sx * center.x() + tx << " " << sy * center.y() + ty << " " << sr * radius << " "
        << startAngleD << " " << endAngleD << " arc" << (closed ? " closepath" : "") << std::endl;
    os << (filled ? "filled" : "stroke") << std::endl;
}

void ps_footer (std::ostream& os) {
    os << "showpage" << std::endl;
}

#endif

