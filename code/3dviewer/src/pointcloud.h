#ifndef _POINTCLOUD_H_
#define _POINTCLOUD_H_

#include <vector>
#include <fstream>
#include <cassert>

#include "misc.h"

template <typename Kernel>
class CPoint_cloud {
    public:
        typedef typename Kernel::Point_3 Point;

    public:
        // LIFE CYCLE ============================================
        CPoint_cloud () {
        }

        template <typename InputIterator>
        CPoint_cloud (InputIterator begin,
                     InputIterator beyond) {
            addPoints(begin, beyond);
        }

        // ADD / REMOVE POINTS ===================================
        void push_back (Point const& p) {
            points.push_back(p);
        }

        template <typename InputIterator>
        void addPoints (InputIterator begin,
                        InputIterator beyond) {
            points.insert(points.end(), begin, beyond);
        }

        size_t size () const {
            return points.size();
        }

        void clear () {
            points.clear();
        }

        // ITERATORS =============================================

        typename std::vector<Point>::iterator begin () {
            return points.begin();
        }

        typename std::vector<Point>::iterator end () {
            return points.end();
        }

        // RENDERING =============================================

        // draw point cloud
        void gl_draw_points (const float point_size,
                             const unsigned char r,
                             const unsigned char g,
                             const unsigned char b) {
            ::glBegin(GL_POINTS);

            ::glPointSize(point_size);
            ::glColor3ub(r, g, b);

            for (size_t i = 0; i < points.size(); ++i) {
                const Point& p = points[i];
                ::glVertex3d(p.x(), p.y(), p.z());
            }

            ::glEnd();
        }

    private:
        std::vector<Point> points;
};

// I/O =================================================

template <typename Kernel>
std::istream& operator>> (std::istream& in, CPoint_cloud<Kernel>& cloud) {
    typedef typename CPoint_cloud<Kernel>::Point Point;

    std::string str;

    while (std::getline(in, str)) {
        std::vector<std::string> vec = misc::split(str, ' ');
        /* assert(vec.size() == 3); */
        Point p(misc::strTo<double>(vec[0]),
                misc::strTo<double>(vec[1]),
                misc::strTo<double>(vec[2]));
        cloud.push_back(p);
    }

    return in;
}

#endif // _POINTCLOUD_H_

