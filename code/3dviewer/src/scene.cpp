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
#include <QCheckBox>
#include <QSpinBox>
#include <QDoubleSpinBox>

#include <QGLViewer/qglviewer.h>

#include "halfspace_intersection_with_constructions_3.h"
#include "inclusion_exclusion.hpp"
#include "add_noise.h"
#include "random_sphere_3.h"
#include "GradientUtils.hpp"

// Comment to disable the computation of the intersections of
// the Voronoi cells and the polyhedron
/* #define COMPUTE_INTERSECTIONS */

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

    // intersections
    m_view_intersections = false;

    // parameters
    // radius
    bool ok = false;
    double r;
    while (!ok) {
        r = QInputDialog::getDouble(NULL, "Balls", "Radius",
                                    0.5, 0, 10, 2, &ok);
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

    // vector field
    if (m_view_intersections) {
        for (size_t i = 0; i < m_inter.size(); ++i) {
            gl_draw_edges_tag(m_inter[i], 1.0f, 0, 0, 255);
        }
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
        m_ball.clear();
        m_balls.clear();
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

        compute_balls();
#ifdef COMPUTE_INTERSECTIONS
        compute_intersections();
#endif
    }

    QApplication::restoreOverrideCursor();

    return 0;
}

void Scene::save (QString filename) {
    QFileInfo fileinfo(filename);
    std::ofstream out(filename.toUtf8());

    out << m_pointcloud;
}

void Scene::saveNormals (QString filename) {
    if (m_pointcloud.size() != m_vectorfield.size())
        return;

    QFileInfo fileinfo(filename);
    std::ofstream out(filename.toUtf8());

    for (size_t i = 0; i < m_pointcloud.size(); ++i) {
        Point_cloud::Point p = m_pointcloud[i];
        out << p << " " << m_vectorfield[p] << "\n";
    }
}

void Scene::compute_balls () {
    m_balls.clear();
    m_pointsAD.clear();

    if (! m_ball.empty()) {
        // add polyhedral balls: \forall p, B_N(p, r)
        size_t N = m_pointcloud.size();
        for (size_t i = 0; i < N; ++i) {
            Point_cloud::Point P = m_pointcloud[i];
            Vector_3 pv = P - CGAL::ORIGIN;
            m_pointsAD.push_back(Point_ad(AD(P.x(), 3 * N, 3 * i),
                                          AD(P.y(), 3 * N, 3 * i + 1),
                                          AD(P.z(), 3 * N, 3 * i + 2)));

            std::vector<Plane_3> boundary;
            for (size_t j = 0; j < m_normalsBall.size(); ++j) {
                Vector_3 vv = m_normalsBall[j];
                Plane_3 p(vv.x(), vv.y(), vv.z(),
                          -(pv * vv + m_radius));
                boundary.push_back(p);
            }
            Polyhedron poly;
            CGAL::halfspace_intersection_with_constructions_3(boundary.begin(),
                                                              boundary.end(),
                                                              poly,
                                                              P);
            m_balls.push_back(poly);
        }
    }
}

void Scene::compute_intersections () {
    m_inter.clear();

    typedef CGAL::Delaunay_triangulation_3<Kernel_ad> DT;
    typedef DT::Vertex_handle Vertex_handle;
    typedef Plane_tag<Kernel_ad> Plane_ad;

    DT dt(m_pointsAD.begin(), m_pointsAD.end());

    for (DT::Finite_vertices_iterator it = dt.finite_vertices_begin();
         it != dt.finite_vertices_end();
         ++it) {
        Vertex_handle v(it);
        std::list<Plane_ad> boundary;

        std::list<Vertex_handle> neighbours;
        dt.adjacent_vertices(v, std::back_inserter(neighbours));

        // Voronoi faces
        for (std::list<Vertex_handle>::iterator it = neighbours.begin();
             it != neighbours.end();
             ++it) {
            if (! dt.is_infinite(*it)) {
                Vector_ad p = ((*it)->point() - v->point()) / 2;
                Plane_ad pp(CGAL::midpoint((*it)->point(), v->point()), p);
                pp.tag = false;
                boundary.push_back(pp);
            }
        }

        // Translated polyhedron
        for (std::vector<Vector_ad>::iterator vit = m_normalsBall_ad.begin();
             vit != m_normalsBall_ad.end();
             ++vit) {
            Vector_ad vv = *vit,
                     pv = v->point() - CGAL::ORIGIN;
            Plane_ad p(vv.x(), vv.y(), vv.z(),
                      -(pv * vv + m_radius));
            p.tag = true;
            boundary.push_back(p);
        }

        Polyhedron_tag P;

        CGAL::halfspace_intersection_with_dual_3(boundary.begin(),
                                                 boundary.end(),
                                                 v->point(),
                                                 P);

        m_inter.push_back(P);
    }
}

void Scene::compute_gradients (int method) {
    FT_ad val_ad = 0;
    VectorXd g = Eigen::VectorXd::Zero(3 * m_pointcloud.size()),
             v = Eigen::VectorXd::Zero(3 * m_pointcloud.size());

    // Convert the point cloud to a big vector
    VectorXd_ad x = container_to_vector<VectorXd_ad>(m_pointcloud.begin(), m_pointcloud.end());

    if (method == 0) { // Volume
        std::cout << "Volume" << std::endl;
        VolumeUnion_ad f(m_radius);
        val_ad = f(x, m_normalsBall_ad.begin(), m_normalsBall_ad.end());
        g = f.grad();
        // TODO: remove finite differences
        /* VectorXd_ad gg = grad_finite_1(f, x, m_normalsBall_ad.begin(), m_normalsBall_ad.end()); */
        /* for (int i = 0; i < gg.rows(); ++i) { */
        /*     g(i) = gg(i).value(); */
        /* } */
        v = f.values();
    } else if (method == 1) { // Area of the boundary naive
        std::cout << "Area of the boundary Naive" << std::endl;
        AreaBoundaryUnion_ad f(m_radius);
        val_ad = f(m_pointcloud.begin(), m_pointcloud.end(),
                   m_normalsBall_ad.begin(), m_normalsBall_ad.end());
        g = f.grad();
    } else if (method == 2) { // Area of the boundary inclusion-exclusion
        std::cout << "Area of the boundary Inclusion-exclusion" << std::endl;
        // TODO: replace by Area_boundary_polyhedron_ad
        val_ad = inclusion_exclusion_minkowski_sum_pointcloud_convex_polyhedron<FT_ad,
               Volume_polyhedron_ad,
               MarkFacesTrue>(m_pointsAD.begin(),
                              m_pointsAD.end(),
                              m_normalsBall_ad.begin(),
                              m_normalsBall_ad.end(),
                              m_radius);
        // TODO: improve
        for (int i = 0; i < g.rows(); ++i) {
            g(i) = val_ad.derivatives().coeffRef(i);
        }
    }

    std::cout << "value: " << val_ad << std::endl;
    for (int i = 0; i < m_pointcloud.size(); ++i) {
        std::cout << m_pointcloud[i] << ": " << v(3 * i) << ", ";
        std::cout << g(3 * i) << ", " << g(3 * i + 1) << ", " << g(3 * i + 2) << std::endl;
    }
    std::ofstream file_gradients("gradients.txt");
    file_gradients << g;

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
    gradientMethod->addItem("Area of the boundary Naive");
    gradientMethod->addItem("Area of the boundary IE");

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
        compute_balls();
    }
}

void Scene::add_noise () {
    bool ok = false;
    double squaredVariance = QInputDialog::getDouble(NULL, "Parameters", "Squared Variance",
                                                     0.01, 0, 1, 4, &ok);

    if (!ok) {
        return;
    }

    std::for_each(m_pointcloud.begin(), m_pointcloud.end(), Add_gaussian_noise(squaredVariance));
    compute_balls();
}

void Scene::random_ellipsoid () {
    QDialog dialog(NULL);
    dialog.setWindowTitle("Points on ellipsoid");
    QFormLayout formLayout(&dialog);

    QSpinBox *numberPoints = new QSpinBox();
    numberPoints->setMinimum(1);
    numberPoints->setMaximum(10000);
    numberPoints->setSingleStep(10);
    numberPoints->setValue(200);
    formLayout.addRow("Number of points:", numberPoints);

    QDoubleSpinBox *xAxis = new QDoubleSpinBox();
    xAxis->setMinimum(0.1);
    xAxis->setMaximum(5);
    xAxis->setSingleStep(0.1);
    xAxis->setValue(0.75);
    formLayout.addRow("a", xAxis);

    QDoubleSpinBox *yAxis = new QDoubleSpinBox();
    yAxis->setMinimum(0.1);
    yAxis->setMaximum(5);
    yAxis->setSingleStep(0.1);
    yAxis->setValue(0.5);
    formLayout.addRow("b", yAxis);

    QDoubleSpinBox *zAxis = new QDoubleSpinBox();
    zAxis->setMinimum(0.1);
    zAxis->setMaximum(5);
    zAxis->setSingleStep(0.1);
    zAxis->setValue(0.5);
    formLayout.addRow("c", zAxis);

    QCheckBox *uniform = new QCheckBox("Uniform");
    formLayout.addRow(uniform);
    uniform->setChecked(true);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                                                       Qt::Horizontal, &dialog);
    formLayout.addRow(buttonBox);

    QObject::connect(buttonBox, SIGNAL(accepted()),
                     &dialog, SLOT(accept()));
    QObject::connect(buttonBox, SIGNAL(rejected()),
                     &dialog, SLOT(reject()));

    if (dialog.exec() == QDialog::Accepted) {
        random_ellipsoid_3<Point_3>(numberPoints->value(),
                                    xAxis->value(),
                                    yAxis->value(),
                                    zAxis->value(),
                                    uniform->isChecked(),
                                    m_pointcloud.back_inserter());

        compute_balls();
    }
}

