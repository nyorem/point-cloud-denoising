#include "MainWindow.hpp"
#include "View.hpp"
#include "Consts.hpp"

#include <QApplication>
#include <QPushButton>
#include <QVBoxLayout>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QInputDialog>
#include <QDialog>
#include <QDialogButtonBox>

MainWindow::MainWindow (int w, int h) : QWidget() {
    resize(w, h);
    setWindowTitle("Viewer");

    // Left side: view
    m_view = new View(consts::width_view, consts::height_view, this);

    // Right side: buttons
    m_rightside = new QWidget(this);
    m_rightside->move(consts::width_view, 0);
    m_rightside->resize(consts::width_window - consts::width_view,
                        consts::height_window);

    // Points
    m_pointsButton = new QPushButton("Points", m_rightside);
    m_pointsButton->move((m_rightside->width() - m_pointsButton->width()) / 2, 0);

    // Balls
    m_ballsButton = new QPushButton("Balls", m_rightside);
    m_ballsButton->move((m_rightside->width() - m_ballsButton->width()) / 2, 0);

    // Reset
    m_resetButton = new QPushButton("Reset", m_rightside);
    m_resetButton->move((m_rightside->width() - m_resetButton->width()) / 2, 0);

    // Delaunay Triangulation
    m_delaunayButton = new QPushButton("Delaunay Triangulation", m_rightside);
    m_delaunayButton->move((m_rightside->width() - m_delaunayButton->width()) / 2, 0);

    // Voronoi vertices
    m_voronoiVerticesButton = new QPushButton("Voronoi vertices", m_rightside);
    m_voronoiVerticesButton->move((m_rightside->width() - m_voronoiVerticesButton->width()) / 2, 0);

    // Points on an ellipse
    m_randomEllipseButton = new QPushButton("Points on ellipse", m_rightside);
    m_randomEllipseButton->move((m_rightside->width() - m_randomEllipseButton->width()) / 2, 0);

    // One step of the algorithm
    m_oneStepButton = new QPushButton("One step", m_rightside);
    m_oneStepButton->move((m_rightside->width() - m_oneStepButton->width()) / 2, 0);

    // Layout
    QVBoxLayout *layout = new QVBoxLayout();
    layout->addWidget(m_resetButton);
    layout->addWidget(m_pointsButton);
    layout->addWidget(m_randomEllipseButton);
    layout->addWidget(m_delaunayButton);
    layout->addWidget(m_voronoiVerticesButton);
    layout->addWidget(m_ballsButton);
    layout->addWidget(m_oneStepButton);
    m_rightside->setLayout(layout);

    // Slots
    // Points
    connect(m_pointsButton, &QPushButton::clicked,
            this, &MainWindow::togglePoints);

    // Balls
    connect(m_ballsButton, &QPushButton::clicked,
            this, &MainWindow::toggleBalls);

    // Reset
    connect(m_resetButton, &QPushButton::clicked,
            this, &MainWindow::resetScene);

    // Delaunay Triangulation
    connect(m_delaunayButton, &QPushButton::clicked,
            this, &MainWindow::toggleDelaunayTriangulation);

    // Voronoi vertices
    connect(m_voronoiVerticesButton, &QPushButton::clicked,
            this, &MainWindow::toggleVoronoiVertices);

    // Points on ellipse
    connect(m_randomEllipseButton, &QPushButton::clicked,
            this, &MainWindow::randomPointsEllipse);

    // One step
    // TODO

    // Parameters initialization
    // Ball radius
    double radius = QInputDialog::getDouble(this, "Parameters for balls", "Radius",
                                            20.0, 0, 100);
    m_view->m_scene->setBallRadius(radius);
}

// Slots
void MainWindow::togglePoints () {
    m_view->m_scene->togglePoints();
}

void MainWindow::toggleBalls () {
    m_view->m_scene->toggleBalls();
}

void MainWindow::resetScene () {
    m_view->m_scene->reset();
}

void MainWindow::toggleDelaunayTriangulation () {
    m_view->m_scene->toggleDelaunayTriangulation();
}

void MainWindow::toggleVoronoiVertices () {
    m_view->m_scene->toggleVoronoiVertices();
}

void MainWindow::oneStep () {
    // TODO
}

void MainWindow::randomPointsEllipse () {
    QDialog dialog(this);
    dialog.setWindowTitle("Parameters");
    QFormLayout formLayout(&dialog);

    QSpinBox *numberPoints = new QSpinBox();
    numberPoints->setMinimum(1);
    numberPoints->setMaximum(100000);
    numberPoints->setSingleStep(10);
    numberPoints->setValue(100);
    formLayout.addRow("Number of points:", numberPoints);

    QDoubleSpinBox *majorAxis = new QDoubleSpinBox();
    majorAxis->setMinimum(1.5);
    majorAxis->setMaximum(100);
    majorAxis->setSingleStep(0.1);
    majorAxis->setValue(1);

    QDoubleSpinBox *minorAxis = new QDoubleSpinBox();
    minorAxis->setMinimum(1);
    minorAxis->setMaximum(100);
    minorAxis->setSingleStep(0.1);
    minorAxis->setValue(1);

    QDoubleSpinBox *noiseVariance = new QDoubleSpinBox();
    noiseVariance->setMinimum(0);
    noiseVariance->setMaximum(200);
    noiseVariance->setSingleStep(1);
    noiseVariance->setValue(0);

    formLayout.addRow("Major axis:", majorAxis);
    formLayout.addRow("Minor axis:", minorAxis);
    formLayout.addRow("Noise variance:", noiseVariance);

    QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                               Qt::Horizontal, &dialog);
    formLayout.addRow(&buttonBox);

    connect(&buttonBox, &QDialogButtonBox::accepted,
            &dialog, &QDialog::accept);
    connect(&buttonBox, &QDialogButtonBox::rejected,
            &dialog, &QDialog::reject);

    if (dialog.exec() == QDialog::Accepted) {
        m_view->m_scene->randomPointsEllipse(numberPoints->value(),
                                             majorAxis->value() * 10,
                                             minorAxis->value() * 10,
                                             noiseVariance->value() * 10);
    }
}

