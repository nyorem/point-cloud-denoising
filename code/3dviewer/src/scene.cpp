#include "scene.h"

#include <iostream>
#include <fstream>

#include <QFileInfo>
#include <QTextStream>
#include <QInputDialog>
#include <QtCore/qglobal.h>
#include <QApplication>
#include <QFormLayout>
#include <QComboBox>
#include <QDialogButtonBox>

#include <QGLViewer/qglviewer.h>

#include "halfspace_intersection_with_constructions_3.h"
#include "inclusion_exclusion.hpp"

Scene::Scene () {
    // view options
    // ball
    m_view_ball = false;

    // balls
    m_view_edges = true;
    m_view_facets = false;
    m_smooth = false;

    // point cloud
    m_view_pointcloud = true;

    // vector field
    m_view_vectorfield = true;

    // parameters
    // radius
    bool ok = false;
    double r;
    while (!ok) {
        r = QInputDialog::getDouble(NULL, "Balls", "Radius",
                                    2.0, 0, 10, 2, &ok);
    }
    m_radius = r;
}

Scene::~Scene () {
}

void Scene::render () {
    ::glDisable(GL_LIGHTING);

    // ball
    if (m_view_ball) {
        m_ball.gl_draw_edges(1.0f, 128, 128, 128);
        m_ball.gl_draw_facets(true, 128, 128, 128);
    }

    // polyhedrons edges
    if (m_view_edges) {
        for (size_t i = 0; i < m_balls.size(); ++i) {
            m_balls[i].gl_draw_edges(1.0f, 0, 0, 0);
        }
    }

    // polyhedrons facets
    if (m_view_facets) {
        for (size_t i = 0; i < m_balls.size(); ++i) {
            m_balls[i].gl_draw_facets(m_smooth, 0, 0, 255);
        }
    }

    // point cloud
    if (m_view_pointcloud) {
        m_pointcloud.gl_draw_points(2.0f, 0, 0, 0);
    }

    // vector field
    if (m_view_vectorfield) {
        m_vectorfield.gl_draw_field(1.0f, 255, 0, 0);
    }
}

int Scene::open (QString filename) {
    QApplication::setOverrideCursor(QCursor(::Qt::WaitCursor));

    QFileInfo fileinfo(filename);
    std::ifstream in(filename.toUtf8());

    if (!in || !fileinfo.isFile() || ! fileinfo.isReadable()) {
        std::cerr << "unable to open file" << std::endl;
        QApplication::restoreOverrideCursor();
        return -1;
    }

    QString ext = fileinfo.completeSuffix();

    if (ext == "off") {
        // read polyhedral ball B_N(0, 1)
        std::cout << "read polyhedral ball...";
        in >> m_ball;
        std::cout << "done" << std::endl;

        if (!in) {
            std::cerr << "invalid OFF file" << std::endl;
            QApplication::restoreOverrideCursor();
            return -1;
        }

        // compute normals
        m_ball.compute_normals();

        for (Polyhedron::Facet_iterator fit = m_ball.facets_begin();
             fit != m_ball.facets_end();
             ++fit) {
            Vector_3 v = fit->normal();
            m_normalsBall.push_back(v);
            m_normalsBall_ad.push_back(Vector_ad(v.x(), v.y(), v.z()));
        }
    } else if (ext == "xyz") {
        // read point cloud
        std::cout << "read point cloud...";
        in >> m_pointcloud;
        std::cout << "done" << std::endl;

        if (! m_ball.empty()) {
            // add polyhedral balls: \forall p, B_N(p, r)
            int i = 0;
            size_t N = m_pointcloud.size();
            for (std::vector<Point_3>::iterator pit = m_pointcloud.begin();
                 pit != m_pointcloud.end();
                 ++pit) {
                m_pointsAD.push_back(Point_ad(AD(pit->x(), 3 * N, 3 * i),
                                              AD(pit->y(), 3 * N, 3 * i + 1),
                                              AD(pit->z(), 3 * N, 3 * i + 2)));
                ++i;

                std::vector<Plane_3> boundary;
                for (size_t i = 0; i < m_normalsBall.size(); ++i) {
                    Vector_3 vv = m_normalsBall[i],
                             pv = *pit - CGAL::ORIGIN;
                    Plane_3 p(vv.x(), vv.y(), vv.z(),
                              -(pv * vv + m_radius));
                    boundary.push_back(p);
                }
                Polyhedron P;
                CGAL::halfspace_intersection_with_constructions_3(boundary.begin(),
                                                                  boundary.end(),
                                                                  P,
                                                                  *pit);
                P.compute_normals();
                m_balls.push_back(P);
            }
        }
    }

    QApplication::restoreOverrideCursor();

    return 0;
}

void Scene::compute_gradients (int method) {
    FT_ad val_ad;
    VectorXd g = Eigen::VectorXd::Zero(3 * m_pointcloud.size());

    if (method == 0) { // Volume
        std::cout << "Volume" << std::endl;
        VolumeUnion_ad f(m_radius);
        val_ad = f(m_pointcloud.begin(), m_pointcloud.end(),
                   m_normalsBall_ad.begin(), m_normalsBall_ad.end());
        g = f.grad();
    } else if (method == 1) { // Area of the boundary 1
        std::cout << "Area of the boundary 1" << std::endl;
        AreaBoundaryUnion_ad f(m_radius);
        val_ad = f(m_pointcloud.begin(), m_pointcloud.end(),
                   m_normalsBall_ad.begin(), m_normalsBall_ad.end());
        g = f.grad();
    } else if (method == 2) { // Area of the boundary 2: exclusion-inclusion
        std::cout << "Area of the boundary 2" << std::endl;
        val_ad = area_boundary_minkowski_sum_pointcloud_convex_polyhedron<FT_ad>(m_pointsAD.begin(),
                                                                                 m_pointsAD.end(),
                                                                                 m_normalsBall_ad.begin(),
                                                                                 m_normalsBall_ad.end(),
                                                                                 m_radius);
        // TODO: do better
        for (int i = 0; i < g.rows(); ++i) {
            g(i) = val_ad.derivatives().coeffRef(i);
        }
    }

    std::cout << "value: " << val_ad << std::endl;
    std::cout << g << std::endl;

    std::vector<Vector_3> grads;
    vector_to_container<Vector_3>(g, std::back_inserter(grads));
    m_vectorfield.clear();
    m_vectorfield.addVectors(m_pointcloud.begin(), m_pointcloud.end(),
                             grads.begin(), grads.end());
}

bool Scene::ask_method (int& method) {
    // Choose the method of the computation of the gradients
    QDialog dialog(NULL);
    dialog.setWindowTitle("Gradient method");
    QFormLayout formLayout(&dialog);

    QComboBox *gradientMethod = new QComboBox();
    gradientMethod->addItem("Volume");
    gradientMethod->addItem("Area of the boundary 1");
    gradientMethod->addItem("Area of the boundary 2");

    formLayout.addRow(gradientMethod);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                                                       Qt::Horizontal, &dialog);
    formLayout.addRow(buttonBox);

    QObject::connect(buttonBox, SIGNAL(accepted()),
                     &dialog, SLOT(accept()));
    QObject::connect(buttonBox, SIGNAL(rejected()),
                     &dialog, SLOT(reject()));

    if (dialog.exec() == QDialog::Accepted) {
        method = gradientMethod->currentIndex();

        return true;
    }

    return false;
}

void Scene::vector_field () {
    int method = 0;

    if (ask_method(method)) {
        compute_gradients(method);
    }
}

void Scene::nsteps () {
    // Number of steps
    bool ok = false;
    int N = QInputDialog::getInt(NULL, "Parameters", "Number of steps",
                                 1, 0, 100, 1, &ok);

    if (!ok) {
        return;
    }

    // Timestep
    ok = false;
    double timestep = QInputDialog::getDouble(NULL, "Parameters", "Timestep",
                                              0.01, 0, 100, 3, &ok);

    if (!ok) {
        return;
    }

    // Method
    int method = 0;
    if (!ask_method(method)) {
        return;
    }

    for (int i = 1; i <= N; ++i) {
        compute_gradients(method);
        std::vector<Point_3> new_points;

        for (int p = 0; p < m_pointcloud.size(); ++p) {
            Point_3 pp = m_pointcloud[p];
            Vector_3 grad = m_vectorfield[pp];
            new_points.push_back(pp - timestep * grad);
        }

        m_pointcloud.clear();
        m_pointcloud.addPoints(new_points.begin(), new_points.end());
    }
}

