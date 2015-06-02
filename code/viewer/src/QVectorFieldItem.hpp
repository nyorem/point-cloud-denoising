#ifndef _QVECTORFIELDITEM_HPP_
#define _QVECTORFIELDITEM_HPP_

#include <QGraphicsItem>
#include <QPainter>
#include <map>

#include <CGAL/Bbox_2.h>

#include "misc.hpp"

// Displaying a vector field: a vector is associated to a 2D point.
// The field may be either coloured uniformly (same colour) or according
// to the norm of the vector.
class QVectorFieldItem : public QGraphicsItem {
    public:
        QVectorFieldItem (const QPen &pen,
                          bool uniform = false,
                          QGraphicsItem *parent = 0) : QGraphicsItem(parent),
                                                       m_pen(pen),
                                                       m_uniform(uniform),
                                                       m_maxNorm(0) {}

        void insert (Point_2 p, Vector_2 v) {
            m_field[p] = v;
            m_points.push_back(p);
            if (v.squared_length() >= m_maxNorm * m_maxNorm) {
                m_maxNorm = CGAL::sqrt(v.squared_length());
            }
        }

        template <typename PointInputIterator, typename VectorInputIterator>
        void insert (PointInputIterator pbegin,
                     PointInputIterator pbeyond,
                     VectorInputIterator vbegin,
                     VectorInputIterator vbeyond) {
            PointInputIterator pit = pbegin;
            VectorInputIterator vit = vbegin;

            while (pit != pbeyond && vit != vbeyond) {
                m_field[*pit] = *vit;

                if (vit->squared_length() >= m_maxNorm * m_maxNorm) {
                    m_maxNorm = CGAL::sqrt(vit->squared_length());
                }

                ++pit;
                ++vit;
            }

            m_points.insert(m_points.begin(), pbegin, pbeyond);
        }

        Vector_2 get (Point_2 const& p) {
            return m_field[p];
        }

        void clear () {
            m_points.clear();
            m_field.clear();
            m_maxNorm = 0;
        }

        void paint (QPainter *painter,
                    const QStyleOptionGraphicsItem *option,
                    QWidget *widget) {
            if (m_uniform) {
                painter->setPen(m_pen);
            }

            for (MapIterator mit = m_field.begin();
                 mit != m_field.end();
                 ++mit) {
                Point_2 p = mit->first;
                Vector_2 v = mit->second;
                // Normalize the vector
                /* v = v / CGAL::sqrt(v.squared_length()); */

                // area unweighted
                /* Point_2 q = p + 20 * v; */
                // square
                /* Point_2 q = p + 2 * v; */

                // area weighted
                /* Point_2 q = p + 1000 * v; */

                // perimeter unweighted
                Point_2 q = p + 100 * v;
                // square
                /* Point_2 q = p + 10 * v; */

                // perimeter weighted
                /* Point_2 q = p + 1000 * v; */

                // area weighted perimeter
                /* Point_2 q = p + 50 * v; */

                double r, g, b;
                misc::ramp(CGAL::sqrt(v.squared_length()) / m_maxNorm, r, g, b);
                if (!m_uniform) {
                    painter->setPen(QColor(r * 255, g * 255, b * 255));
                }
                painter->drawLine(p.x(), p.y(), q.x(), q.y());
            }
        }

        QRectF boundingRect () const {
            CGAL::Bbox_2 b = CGAL::bbox_2(m_points.begin(), m_points.end());

            QPointF topLeft(b.xmin(), b.ymax()),
                    bottomRight(b.xmax(), b.ymin());

            return QRectF(topLeft, bottomRight);
        }

        ~QVectorFieldItem () {
            clear();
        }

    private:
        QPen m_pen;
        bool m_uniform;
        std::map<Point_2, Vector_2> m_field;
        Points_2 m_points;
        double m_maxNorm;

        typedef std::map<Point_2, Vector_2>::const_iterator MapIterator;
};

#endif

