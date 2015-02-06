#ifndef _QTRIANGULATIONITEM_HPP_
#define _QTRIANGULATIONITEM_HPP_

#include <QGraphicsItem>
#include <QPainter>

#include <CGAL/Bbox_2.h>
#include <vector>

// Wrapper for displaying a CGAL 2D triangulation
template <typename T>
class QTriangulationItem : public QGraphicsItem {
    public:
        typedef typename T::Point Point_2;
        typedef typename T::Segment Segment_2;

        QTriangulationItem (const QPen &pen,
                            QGraphicsItem *parent = 0) : QGraphicsItem(parent), pen(pen) {}

        void insert (Point_2 p) {
            tri.insert(p);
            m_points.push_back(p);
        }

        template <typename InputIterator>
        void insert (InputIterator begin,
                     InputIterator beyond) {
            tri.insert(begin, beyond);
            m_points.insert(m_points.begin(), begin, beyond);
        }

        void paint (QPainter *painter,
                    const QStyleOptionGraphicsItem *option,
                    QWidget *widget) {
            painter->setPen(pen);

            for (typename T::Finite_edges_iterator eit = tri.finite_edges_begin();
                 eit != tri.finite_edges_end();
                 ++eit) {
                Segment_2 s = tri.segment(*eit);
                Point_2 p = s.source(), q = s.target();
                painter->drawLine(p.x(), p.y(), q.x(), q.y());
            }

            QGraphicsItem::scene()->update();
        }

        QRectF boundingRect () const {
            CGAL::Bbox_2 b = CGAL::bbox_2(m_points.begin(), m_points.end());

            QPointF topLeft(b.xmin(), b.ymax()),
                    bottomRight(b.xmax(), b.ymin());

            return QRectF(topLeft, bottomRight);
        }

        ~QTriangulationItem () {
            tri.clear();
            m_points.clear();
        }

    protected:
        T tri;
        std::vector<Point_2> m_points;
        QPen pen;
};

#endif

