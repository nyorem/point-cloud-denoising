#ifndef _QDELAUNAYTRIANGULATIONITEM_HPP_
#define _QDELAUNAYTRIANGULATIONITEM_HPP_

#include "QTriangulationItem.hpp"

// Wrapper for displaying a CGAL 2D Delaunay triangulation
// and its Voronoi vertices
template <typename DT>
class QDelaunayTriangulationItem : public QTriangulationItem<DT> {
    public:
        typedef typename DT::Point Point_2;
        typedef typename DT::Segment Segment_2;

        QDelaunayTriangulationItem (const QPen &pen,
                                    const QPen &penVoronoiVertices,
                                    QGraphicsItem *parent = 0) : QTriangulationItem<DT>(pen, parent),
                                                                 m_voronoiVerticesVisible(false),
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
        }

        ~QDelaunayTriangulationItem () {
            m_voronoiVertices.clear();
        }

    private:
        bool m_voronoiVerticesVisible;
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

