#include "Scene.hpp"
#include "Graphics.hpp"
#include "random_square_2.hpp"
#include "random_ellipse_2.hpp"

#include "CGAL_AD_typedefs.hpp"
#include "GradientUtils.hpp"
#include "PointCloudUtils.hpp"

#include <iterator>

Scene::Scene (QObject *parent) : QGraphicsScene(parent) {
    init();
}

void Scene::init () {
    // Points
    m_points = new QPointListItem(Graphics::solidBlack);
    addItem(m_points);
    m_points->show();

    // Delaunay Triangulation and Voronoi vertices
    m_dt = new QDelaunayTriangulation2Item(Graphics::solidRed, Graphics::solidBlue);
    addItem(m_dt);
    m_dt->hide();

    // Balls (offset)
    m_balls = new QPointListItem(Graphics::solidRed, true);
    addItem(m_balls);
    m_balls->hide();

    // Gradients
    m_gradients = new QVectorFieldItem(Graphics::solidBlue);
    addItem(m_gradients);
    m_gradients->hide();
}

void Scene::addPoint (int x, int y) {
    m_points->insert(Point_2(x, y));
    m_dt->insert(Point_2(x, y));
    m_balls->insert(Point_2(x, y));
}

void Scene::setBallRadius (float radius) {
    m_balls->setRadius(radius);
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
    // TODO: dialog box for radius / timestep
    // TODO: use new solver / volume
    VolumeUnion_ad volume_union_ad(1);

    VectorXd_ad points_vec = pointCloudToVector<VectorXd_ad>(m_points->begin(), m_points->end());
    GradAdEval<FT_ad, Function_ad, VectorXd_ad> grad_ad_eval;
    VectorXd_ad new_points_vec = step_gradient_descent(grad_ad_eval, volume_union_ad, points_vec, 0.1);
    Points_2 new_points;
    vectorToPointCloud<Point_2>(toValue(new_points_vec), std::back_inserter(new_points));

    // New point cloud
    m_points->clear();
    m_points->insert(new_points.begin(), new_points.end());

    // New gradients
    // TODO
    Eigen::VectorXd gradients = volume_union_ad.grad();
}

void Scene::toggleGradients () {
    if (m_gradients->isVisible()) {
        m_gradients->hide();
    } else {
        m_gradients->show();
        update();
    }
}

void Scene::reset () {
    clear();

    // Re-intializes the scene
    init();
}

