#ifndef _QPOINTLISTITEM_HPP_
#define _QPOINTLISTITEM_HPP_

#include <QGraphicsItem>
#include <QPainter>

#include <CGAL/Bbox_2.h>

#include "CGAL_typedefs.hpp"

// Wrapper for displaying a list of points
class QPointListItem : public QGraphicsItem {
    public:
        QPointListItem (const QPen& pen,
                        bool drawBalls = false,
                        float radius = 2.0,
                        QGraphicsItem *parent = 0) : QGraphicsItem(parent), m_radius(radius),
                                                     m_pen(pen), m_drawBalls(drawBalls) {}

        void insert (Point_2 p) {
            m_points.push_back(p);
        }

        template <typename InputIterator>
        void insert (InputIterator begin,
                     InputIterator beyond) {
            m_points.insert(m_points.begin(), begin, beyond);
        }

        void setRadius (float radius) {
            m_radius = radius;
        }

        float radius () const {
            return m_radius;
        }

        void drawPoint (QPainter *painter, Point_2 const& p) {
            QPointF orig(p.x(), p.y()),
                    dx(m_radius, 0.0),
                    dy(0.0, m_radius);

            painter->drawLine(orig - dx, orig + dx);
            painter->drawLine(orig - dy, orig + dy);
        }

        void drawCircle (QPainter *painter, Point_2 const& p) {
            QPointF center(p.x(), p.y());

            painter->drawEllipse(center, m_radius, m_radius);
        }

        void drawFilledCircle (QPainter *painter, Point_2 const &p) {
            QPointF center(p.x(), p.y());

            painter->setBrush(QBrush(m_pen.color()));
            painter->drawEllipse(center, m_radius, m_radius);
        }

        void paint (QPainter *painter,
                    const QStyleOptionGraphicsItem *option,
                    QWidget *widget) {
            painter->setPen(m_pen);

            for (int i = 0; i < m_points.size(); ++i) {
                if (m_drawBalls) {
                    drawFilledCircle(painter, m_points[i]);
                } else {
                    drawPoint(painter, m_points[i]);
                }
            }

            QGraphicsItem::scene()->update();
        }

        QRectF boundingRect () const {
            CGAL::Bbox_2 b = CGAL::bbox_2(m_points.begin(), m_points.end());

            QPointF topLeft(b.xmin(), b.ymax()),
                    bottomRight(b.xmax(), b.ymin());

            return QRectF(topLeft, bottomRight);
        }

        ~QPointListItem () {
            m_points.clear();
        }

    private:
        Points_2 m_points;
        float m_radius;
        QPen m_pen;
        bool m_drawBalls;
};

#endif

