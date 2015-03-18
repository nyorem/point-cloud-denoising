#ifndef _QVECTORFIELDITEM_HPP_
#define _QVECTORFIELDITEM_HPP_

#include <QGraphicsItem>
#include <QPainter>

#include <CGAL/Bbox_2.h>

#include <map>

// Displaying a vector field: a vector is associated to a 2D point
class QVectorFieldItem : public QGraphicsItem {
    public:
        QVectorFieldItem (const QPen &pen,
                          QGraphicsItem *parent = 0) : QGraphicsItem(parent),
                                                       m_pen(pen) {}

        void insert (Point_2 p, Vector_2 v) {
            m_field[p] = v;
            m_points.push_back(p);
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

                ++pit;
                ++vit;
            }

            m_points.insert(m_points.begin(), pbegin, pbeyond);
        }

        void clear () {
            m_field.clear();
        }

        void paint (QPainter *painter,
                    const QStyleOptionGraphicsItem *option,
                    QWidget *widget) {
            painter->setPen(m_pen);

            for (MapIterator mit = m_field.begin();
                 mit != m_field.end();
                 ++mit) {
                Point_2 p = mit->first;
                Vector_2 v = mit->second;
                // Normalize the vector
                // TODO
                /* v = v / CGAL::sqrt(v.squared_length()); */
                /* v = 50 * v; */
                /* Point_2 q = p + v; */
                Point_2 q = p + 50 * v;

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
            m_field.clear();
            m_points.clear();
        }

    private:
        QPen m_pen;
        std::map<Point_2, Vector_2> m_field;
        Points_2 m_points;

        typedef std::map<Point_2, Vector_2>::const_iterator MapIterator;
};

#endif

