#include "MainWindow.hpp"
#include "View.hpp"
#include "Consts.hpp"

#include <QApplication>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSpinBox>
#include <QCheckBox>
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

    // Save
    m_saveButton = new QPushButton("Save", m_rightside);
    m_saveButton->move((m_rightside->width() - m_saveButton->width()) / 2, 0);

    // Save
    m_loadButton = new QPushButton("Load", m_rightside);
    m_loadButton->move((m_rightside->width() - m_loadButton->width()) / 2, 0);

    // Delaunay Triangulation
    m_delaunayButton = new QPushButton("Delaunay Triangulation", m_rightside);
    m_delaunayButton->move((m_rightside->width() - m_delaunayButton->width()) / 2, 0);

    // Voronoi vertices
    m_voronoiVerticesButton = new QPushButton("Voronoi vertices", m_rightside);
    m_voronoiVerticesButton->move((m_rightside->width() - m_voronoiVerticesButton->width()) / 2, 0);

    // Voronoi edges
    m_voronoiEdgesButton = new QPushButton("Voronoi edges", m_rightside);
    m_voronoiEdgesButton->move((m_rightside->width() - m_voronoiEdgesButton->width()) / 2, 0);

    // Points on an ellipse
    m_randomEllipseButton = new QPushButton("Points on ellipse", m_rightside);
    m_randomEllipseButton->move((m_rightside->width() - m_randomEllipseButton->width()) / 2, 0);

    // N steps of the algorithm
    m_nStepsButton = new QPushButton("N steps", m_rightside);
    m_nStepsButton->move((m_rightside->width() - m_nStepsButton->width()) / 2, 0);

    // Compute gradients
    m_computeGradientsButton = new QPushButton("Compute Gradients", m_rightside);
    m_computeGradientsButton->move((m_rightside->width() - m_computeGradientsButton->width()) / 2, 0);

    // Gradients
    m_gradientsButton = new QPushButton("Gradients", m_rightside);
    m_gradientsButton->move((m_rightside->width() - m_gradientsButton->width()) / 2, 0);

    // Decomposition
    m_decompositionButton = new QPushButton("Decomposition", m_rightside);
    m_decompositionButton->move((m_rightside->width() - m_decompositionButton->width()) / 2, 0);

    // Layout
    QVBoxLayout *layout = new QVBoxLayout();
    layout->addWidget(m_resetButton);
    layout->addWidget(m_saveButton);
    layout->addWidget(m_loadButton);
    layout->addWidget(m_pointsButton);
    layout->addWidget(m_randomEllipseButton);
    layout->addWidget(m_delaunayButton);
    layout->addWidget(m_voronoiVerticesButton);
    layout->addWidget(m_voronoiEdgesButton);
    layout->addWidget(m_ballsButton);
    layout->addWidget(m_nStepsButton);
    layout->addWidget(m_computeGradientsButton);
    layout->addWidget(m_gradientsButton);
    layout->addWidget(m_decompositionButton);
    m_rightside->setLayout(layout);

    // Slots
    // Points
    /* connect(m_pointsButton, &QPushButton::clicked, */
    /*         this, &MainWindow::togglePoints); */
    connect(m_pointsButton, SIGNAL(clicked()),
            this, SLOT(togglePoints()));

    // Balls
    /* connect(m_ballsButton, &QPushButton::clicked, */
    /*         this, &MainWindow::toggleBalls); */
    connect(m_ballsButton, SIGNAL(clicked()),
            this, SLOT(toggleBalls()));

    // Reset
    /* connect(m_resetButton, &QPushButton::clicked, */
    /*         this, &MainWindow::resetScene); */
    connect(m_resetButton, SIGNAL(clicked()),
            this, SLOT(resetScene()));

    // Save
    /* connect(m_saveButton, &QPushButton::clicked, */
    /*         this, &MainWindow::savePointCloud); */
    connect(m_saveButton, SIGNAL(clicked()),
            this, SLOT(savePointCloud()));

    // Load
    /* connect(m_loadButton, &QPushButton::clicked, */
    /*         this, &MainWindow::loadPointCloud); */
    connect(m_loadButton, SIGNAL(clicked()),
            this, SLOT(loadPointCloud()));

    // Delaunay Triangulation
    /* connect(m_delaunayButton, &QPushButton::clicked, */
    /*         this, &MainWindow::toggleDelaunayTriangulation); */
    connect(m_delaunayButton, SIGNAL(clicked()),
            this, SLOT(toggleDelaunayTriangulation()));

    // Voronoi vertices
    /* connect(m_voronoiVerticesButton, &QPushButton::clicked, */
    /*         this, &MainWindow::toggleVoronoiVertices); */
    connect(m_voronoiVerticesButton, SIGNAL(clicked()),
            this, SLOT(toggleVoronoiVertices()));

    // Voronoi edges
    /* connect(m_voronoiEdgesButton, &QPushButton::clicked, */
    /*         this, &MainWindow::toggleVoronoiEdges); */
    connect(m_voronoiEdgesButton, SIGNAL(clicked()),
            this, SLOT(toggleVoronoiEdges()));

    // Points on ellipse
    /* connect(m_randomEllipseButton, &QPushButton::clicked, */
    /*         this, &MainWindow::randomPointsEllipse); */
    connect(m_randomEllipseButton, SIGNAL(clicked()),
            this, SLOT(randomPointsEllipse()));

    // One step
    /* connect(m_nStepsButton, &QPushButton::clicked, */
    /*         this, &MainWindow::nSteps); */
    connect(m_nStepsButton, SIGNAL(clicked()),
            this, SLOT(nSteps()));

    // Gradients
    /* connect(m_gradientsButton, &QPushButton::clicked, */
    /*         this, &MainWindow::toggleGradients); */
    connect(m_gradientsButton, SIGNAL(clicked()),
            this, SLOT(toggleGradients()));

    // Compute Gradients
    /* connect(m_computeGradientsButton, &QPushButton::clicked, */
    /*         this, &MainWindow::computeGradients); */
    connect(m_computeGradientsButton, SIGNAL(clicked()),
            this, SLOT(computeGradients()));

    // Decomposition
    /* connect(m_decompositionButton, &QPushButton::clicked, */
    /*         this, &MainWindow::toggleDecomposition); */
    connect(m_decompositionButton, SIGNAL(clicked()),
            this, SLOT(toggleDecomposition()));

    // Hack to bring the window to the front
    /* show(); */
    /* raise(); */
    /* activateWindow(); */

    // Parameters initialization
    // Ball radius
    double radius = QInputDialog::getDouble(this, "Balls", "Radius",
                                            15.0, 0, 1000);
    m_view->m_scene->setBallRadius(radius);

    // Timestep for gradient descent
    double timestep = QInputDialog::getDouble(this, "Gradient descent", "Timestep",
                                              0.5, 0, 100, 3);
    m_view->m_scene->setTimestep(timestep);
}

void MainWindow::resizeEvent (QResizeEvent* event) {
    int w = event->size().width(),
        h = event->size().height();

    resize(w, h);
    m_view->resize(w - 200, h);

    m_rightside->move(w - 200, 0);
    m_rightside->resize(200, h);
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

void MainWindow::savePointCloud () {
    m_view->m_scene->savePointCloud();
}

void MainWindow::loadPointCloud () {
    m_view->m_scene->loadPointCloud();
}

void MainWindow::toggleDelaunayTriangulation () {
    m_view->m_scene->toggleDelaunayTriangulation();
}

void MainWindow::toggleVoronoiVertices () {
    m_view->m_scene->toggleVoronoiVertices();
}

void MainWindow::toggleVoronoiEdges () {
    m_view->m_scene->toggleVoronoiEdges();
}

void MainWindow::nSteps () {
    m_view->m_scene->nSteps();
}

void MainWindow::toggleGradients () {
    m_view->m_scene->toggleGradients();
}

void MainWindow::computeGradients () {
    m_view->m_scene->computeGradients();
}

void MainWindow::toggleDecomposition () {
    m_view->m_scene->toggleDecomposition();
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
    majorAxis->setMinimum(0.5);
    majorAxis->setMaximum(100);
    majorAxis->setSingleStep(0.1);
    majorAxis->setValue(1.5);

    QDoubleSpinBox *minorAxis = new QDoubleSpinBox();
    minorAxis->setMinimum(0.5);
    minorAxis->setMaximum(100);
    minorAxis->setSingleStep(0.1);
    minorAxis->setValue(1);

    QDoubleSpinBox *noiseVariance = new QDoubleSpinBox();
    noiseVariance->setMinimum(0);
    noiseVariance->setMaximum(200);
    noiseVariance->setSingleStep(1);
    noiseVariance->setValue(0);

    QDoubleSpinBox *oscMagnitude = new QDoubleSpinBox();
    oscMagnitude->setMinimum(0);
    oscMagnitude->setMaximum(200);
    oscMagnitude->setSingleStep(5);
    oscMagnitude->setValue(0);

    QCheckBox *uniform = new QCheckBox("Uniform");
    uniform->setChecked(true);

    formLayout.addRow("Major axis:", majorAxis);
    formLayout.addRow("Minor axis:", minorAxis);
    formLayout.addRow("Noise variance:", noiseVariance);
    formLayout.addRow("Oscillation magnitude", oscMagnitude);
    formLayout.addRow(uniform);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                                                       Qt::Horizontal, &dialog);
    formLayout.addRow(buttonBox);

    /* connect(&buttonBox, &QDialogButtonBox::accepted, */
    /*         &dialog, &QDialog::accept); */
    connect(buttonBox, SIGNAL(accepted()),
            &dialog, SLOT(accept()));

    /* connect(&buttonBox, &QDialogButtonBox::rejected, */
    /*         &dialog, &QDialog::reject); */
    connect(buttonBox, SIGNAL(rejected()),
            &dialog, SLOT(reject()));

    if (dialog.exec() == QDialog::Accepted) {
        m_view->m_scene->randomPointsEllipse(numberPoints->value(),
                                             majorAxis->value() * 120,
                                             minorAxis->value() * 120,
                                             noiseVariance->value() * 10,
                                             oscMagnitude->value(),
                                             uniform->isChecked());
    }
}

