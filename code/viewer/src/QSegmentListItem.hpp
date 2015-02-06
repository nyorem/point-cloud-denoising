#ifndef _QSEGMENTLISTITEM_HPP_
#define _QSEGMENTLISTITEM_HPP_

#include <QGraphicsItem>
#include <QPainter>

#include <CGAL/Bbox_2.h>

#include "CGAL_typedefs.hpp"

// Wrapper for displaying a 2D path
class QSegmentListItem : public QGraphicsItem {
    public:
        QSegmentListItem (const QPen& pen,
                          QGraphicsItem *parent = 0) : QGraphicsItem(parent), pen(pen) {}

        void insert (Point_2 p) {
            m_points.push_back(p);

            computeSegments();
        }

        template <typename InputIterator>
        void insert (InputIterator begin,
                     InputIterator beyond) {
            m_points.insert(m_points.begin(), begin, beyond);

            computeSegments();
        }

        void paint (QPainter *painter,
                    const QStyleOptionGraphicsItem *option,
                    QWidget *widget) {
            painter->setPen(pen);

            for (int i = 0; i < m_segments.size(); ++i) {
                Segment_2 s = m_segments[i];
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

        ~QSegmentListItem () {
            m_points.clear();
        }

    protected:
        virtual void computeSegments () = 0;

        Points_2 m_points;
        Segments_2 m_segments;
        QPen pen;
};

#endif

