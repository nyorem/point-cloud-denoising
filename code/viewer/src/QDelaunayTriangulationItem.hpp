#ifndef _QDELAUNAYTRIANGULATIONITEM_HPP_
#define _QDELAUNAYTRIANGULATIONITEM_HPP_

#include "QTriangulationItem.hpp"

// Wrapper for displaying a CGAL 2D Delaunay triangulation
// and its Voronoi vertices / edges.
template <typename DT>
class QDelaunayTriangulationItem : public QTriangulationItem<DT> {
    public:
        typedef typename DT::Point Point_2;
        typedef typename DT::Segment Segment_2;
        typedef typename CGAL::Kernel_traits<Point_2>::Kernel Kernel;
        typedef typename Kernel::Ray_2 Ray_2;

        QDelaunayTriangulationItem (const QPen &pen,
                                    const QPen &penVoronoiVertices,
                                    QGraphicsItem *parent = 0) : QTriangulationItem<DT>(pen, parent),
                                                                 m_voronoiVerticesVisible(false),
                                                                 m_voronoiEdgesVisible(false),
                                                                 penVoronoiVertices(penVoronoiVertices) {}

        void insert (Point_2 p) {
            QTriangulationItem<DT>::insert(p);
            computeVoronoiVertices();
        }

        template <typename InputIterator>
        void insert (InputIterator begin,
                     InputIterator beyond) {
            QTriangulationItem<DT>::insert(begin, beyond);
            computeVoronoiVertices();
        }

        bool isVoronoiVerticesVisible () const {
            return m_voronoiVerticesVisible;
        }

        void showVoronoiVertices () {
            m_voronoiVerticesVisible = true;
        }

        void hideVoronoiVertices () {
            m_voronoiVerticesVisible = false;
        }

        bool isVoronoiEdgesVisible () const {
            return m_voronoiEdgesVisible;
        }

        void showVoronoiEdges () {
            m_voronoiEdgesVisible = true;
        }

        void hideVoronoiEdges () {
            m_voronoiEdgesVisible = false;
        }

        void paint (QPainter *painter,
                    const QStyleOptionGraphicsItem *option,
                    QWidget *widget) {
            QTriangulationItem<DT>::paint(painter, option, widget);

            if (isVoronoiVerticesVisible()) {
                painter->setPen(penVoronoiVertices);

                for (int i = 0; i < m_voronoiVertices.size(); ++i) {
                    painter->drawEllipse(m_voronoiVertices[i].x(),
                                         m_voronoiVertices[i].y(),
                                         1, 1);
                }
            }

            if (isVoronoiEdgesVisible()) {
                painter->setPen(penVoronoiVertices);
                for (typename DT::Finite_edges_iterator eit = this->tri.finite_edges_begin();
                     eit != this->tri.finite_edges_end();
                     ++eit) {
                    CGAL::Object vEdge = this->tri.dual(eit);
                    if (const Segment_2* segment = CGAL::object_cast<Segment_2>(&vEdge)) {
                        const Point_2& p1 = segment->source();
                        const Point_2& p2 = segment->target();
                        painter->drawLine(p1.x(), p1.y(), p2.x(), p2.y());
                    } else if (const Ray_2* ray = CGAL::object_cast<Ray_2>(&vEdge)) {
                        const Point_2& p = ray->source();
                        const Point_2 q = p + 100 * ray->to_vector();
                        painter->drawLine(p.x(), p.y(), q.x(), q.y());
                    }
                }
            }
        }

        ~QDelaunayTriangulationItem () {
            m_voronoiVertices.clear();
        }

    private:
        bool m_voronoiVerticesVisible;
        bool m_voronoiEdgesVisible;
        QPen penVoronoiVertices;
        std::vector<Point_2> m_voronoiVertices;

        void computeVoronoiVertices () {
            m_voronoiVertices.clear();

            for (typename DT::Finite_faces_iterator fit = this->tri.finite_faces_begin();
                 fit != this->tri.finite_faces_end();
                 ++fit) {
                typename DT::Face_handle handle = fit;
                Point_2 p = this->tri.dual(handle);
                m_voronoiVertices.push_back(p);
            }
        }

};

#endif

