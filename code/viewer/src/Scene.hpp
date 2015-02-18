#ifndef _PAINTER_HPP_
#define _PAINTER_HPP_

#include <QObject>
#include <QGraphicsScene>

#include "CGAL_typedefs.hpp"
#include "QPointListItem.hpp"
#include "QSegmentListItem.hpp"
#include "QVectorFieldItem.hpp"

class Scene : public QGraphicsScene {
    public:
        Scene (QObject *parent = 0);

        void init ();

        void addPoint (int x, int y);

        void setBallRadius (float radius);
        void setTimestep (double timestep);

        void togglePoints ();
        void toggleBalls ();
        void toggleDelaunayTriangulation ();
        void toggleVoronoiVertices ();
        void toggleVoronoiEdges ();
        void randomPointsEllipse (int N, float a, float b, float noiseVariance);
        void oneStep ();
        void toggleGradients ();
        void toggleDecomposition ();

        void reset ();

    private:
        QPointListItem* m_points;
        QPointListItem* m_balls;
        QDelaunayTriangulation2Item* m_dt;
        QSegmentListItem *m_decomposition;
        QVectorFieldItem *m_gradients;

        double m_timestep;
        double m_radius;
};

#endif

