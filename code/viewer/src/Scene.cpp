#include "Scene.hpp"
#include "Graphics.hpp"
#include "random_square_2.hpp"
#include "random_ellipse_2.hpp"

#include "CGAL_AD_typedefs.hpp"
#include "GradientUtils.hpp"
#include "PointCloudUtils.hpp"
#include "volume_union_balls_2_debug.h"

#include <iterator>

#include <QInputDialog>

Scene::Scene (QObject *parent) : QGraphicsScene(parent) {
    init();
}

void Scene::init () {
    // Points
    m_points = new QPointListItem(Graphics::solidBlack);
    addItem(m_points);
    m_points->show();

    // Delaunay Triangulation and Voronoi vertices / edges
    m_dt = new QDelaunayTriangulation2Item(Graphics::solidBlack, Graphics::solidBlue);
    addItem(m_dt);
    m_dt->hide();

    // Balls (offset)
    m_balls = new QPointListItem(Graphics::solidRed, true);
    addItem(m_balls);
    m_balls->setRadius(m_radius);
    m_balls->hide();

    // Gradients
    m_gradients = new QVectorFieldItem(Graphics::solidBlue);
    addItem(m_gradients);
    m_gradients->hide();

    // Decomposition
    m_decomposition = new QSegmentListItem(Graphics::solidGreen);
    addItem(m_decomposition);
    m_decomposition->hide();
}

void Scene::addPoint (int x, int y) {
    m_points->insert(Point_2(x, y));
    m_dt->insert(Point_2(x, y));
    m_balls->insert(Point_2(x, y));
}

void Scene::setBallRadius (float radius) {
    m_balls->setRadius(radius);
    m_radius = radius;
}

void Scene::setTimestep (double timestep) {
    m_timestep = timestep;
}

void Scene::togglePoints () {
    if (m_points->isVisible()) {
        m_points->hide();
    } else {
        m_points->show();
        update();
    }
}

void Scene::toggleBalls () {
    if (m_balls->isVisible()) {
        m_balls->hide();
    } else {
        m_balls->show();
        update();
    }
}

void Scene::toggleDelaunayTriangulation () {
    if (m_dt->isVisible()) {
        m_dt->hide();
    } else {
        m_dt->show();
        update();
    }
}

void Scene::toggleVoronoiVertices () {
    if (m_dt->isVoronoiVerticesVisible()) {
        m_dt->hideVoronoiVertices();
    } else {
        m_dt->showVoronoiVertices();
        update();
    }
}

void Scene::toggleVoronoiEdges () {
    if (m_dt->isVoronoiEdgesVisible()) {
        m_dt->hideVoronoiEdges();
    } else {
        m_dt->showVoronoiEdges();
        update();
    }
}

void Scene::randomPointsEllipse (int N, float a, float b, float noiseVariance) {
    Points_2 points;
    random_ellipse_2(N, a, b, noiseVariance, std::back_inserter(points));

    m_points->insert(points.begin(), points.end());
    m_balls->insert(points.begin(), points.end());
    m_dt->insert(points.begin(), points.end());
}

void Scene::oneStep () {
    VectorXd_ad points_vec = pointCloudToVector<VectorXd_ad>(m_points->begin(), m_points->end());

    // Compute the volume of the union and the gradient
    VolumeUnion_ad volume(m_radius);
    volume(points_vec);
    Eigen::VectorXd grad = volume.grad();

    // Update the gradients
    std::vector<Vector_2> gradients_vectors;
    vectorToPointCloud<Vector_2>(grad, std::back_inserter(gradients_vectors));
    m_gradients->clear();
    m_gradients->insert(m_points->begin(), m_points->end(),
                        gradients_vectors.begin(), gradients_vectors.end());

    // New point cloud: one step of gradient descent
    // TODO: problem with grad
    /* GradFinite1Eval<Function_ad, VectorXd_ad> grad_finite_1_eval; */
    /* VectorXd_ad new_points_vec = step_gradient_descent(grad_finite_1_eval, volume, points_vec, m_timestep); */
     GradAdEval<FT_ad, Function_ad, VectorXd_ad> grad_ad_eval;
     /* VectorXd_ad new_points_vec = step_gradient_descent(grad_ad_eval, volume, points_vec, m_timestep); */
     VectorXd_ad new_points_vec = gradient_descent(grad_ad_eval, volume, points_vec, m_timestep);
    Points_2 new_points;
    /* int i = 0; */
    /* for (std::vector<Point_2>::const_iterator it = m_points->begin(); */
    /*      it != m_points->end(); */
    /*      ++it) { */
    /*     new_points.push_back(*it - m_timestep * gradients_vectors[i]); */
    /*     i++; */
    /* } */
    vectorToPointCloud<Point_2>(toValue(new_points_vec), std::back_inserter(new_points));
    m_points->clear();
    m_points->insert(new_points.begin(), new_points.end());

    // Update the balls
    m_balls->clear();
    m_balls->insert(m_points->begin(), m_points->end());

    // Update the Delaunay triangulation
    m_dt->clear();
    m_dt->insert(m_points->begin(), m_points->end());

    // Decomposition
    m_decomposition->clear();
    std::vector<Segment_2> segments;
    volume_union_balls_2_debug<FT>(m_points->begin(), m_points->end(), m_radius, segments);
    m_decomposition->insert(segments.begin(), segments.end());
}

void Scene::toggleGradients () {
    if (m_gradients->isVisible()) {
        m_gradients->hide();
    } else {
        m_gradients->show();
        update();
    }
}

void Scene::toggleDecomposition () {
    if (m_decomposition->isVisible()) {
        m_decomposition->hide();
    } else {
        m_decomposition->show();
        update();
    }
}

void Scene::reset () {
    clear();

    // Re-intializes the scene
    init();
}

