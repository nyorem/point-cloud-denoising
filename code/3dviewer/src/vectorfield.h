#ifndef _VECTORFIELD_H_
#define _VECTORFIELD_H_

#include <map>

template <typename Kernel>
class CVector_field {
    public:
        typedef typename Kernel::Point_3 Point;
        typedef typename Kernel::Vector_3 Vector;

    public:
        // LIFE CYCLE ============================================
        CVector_field () {
        }

        template <typename PointIterator, typename VectorIterator>
        CVector_field (PointIterator pbegin,
                       PointIterator pbeyond,
                       VectorIterator vbegin,
                       VectorIterator vbeyond) {
            addPoints(pbegin, pbeyond, vbegin, vbeyond);
        }

        // ADD / REMOVE VECTORS ==================================
        void addVector (Point p, Vector v) {
            vector_field[p] = v;
        }

        template <typename PointIterator, typename VectorIterator>
        void addVectors (PointIterator pbegin,
                         PointIterator pbeyond,
                         VectorIterator vbegin,
                         VectorIterator vbeyond) {
            PointIterator pit = pbegin;
            VectorIterator vit = vbegin;

            while (pit != pbeyond && vit != vbeyond) {
                vector_field[*pit] = *vit;

                ++pit;
                ++vit;
            }
        }

        void clear () {
            vector_field.clear();
        }

        // ACCESS VECTORS ==================================
        Vector& operator[] (Point const& p) {
            return vector_field[p];
        }

        // RENDERING =============================================
        // draw vecor field
        void gl_draw_field (const float line_width,
                            const unsigned char r,
                            const unsigned char g,
                            const unsigned char b) {
            ::glBegin(GL_LINES);

            ::glLineWidth(line_width);
            ::glColor3ub(r, g, b);

            for (typename std::map<Point, Vector>::iterator mit = vector_field.begin();
                 mit != vector_field.end();
                 ++mit) {
                Point p = mit->first;
                Vector v = mit->second;
                v = v / CGAL::sqrt(v.squared_length());
                Point q = p + 0.5 * v;

                ::glVertex3d(p[0], p[1], p[2]);
                ::glVertex3d(q[0], q[1], q[2]);
            }

            ::glEnd();
        }

    private:
        std::map<Point, Vector> vector_field;
};

#endif

