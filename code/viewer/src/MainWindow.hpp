#ifndef _MAINWINDOW_HPP_
#define _MAINWINDOW_HPP_

#include <QPushButton>
#include <QWidget>

#include "Consts.hpp"
#include "View.hpp"

class MainWindow : public QWidget {
    Q_OBJECT

    public:
        MainWindow (int w = consts::width_window,
                    int h = consts::height_window);

        void resizeEvent (QResizeEvent* event);

    public slots:
        void resetScene ();
        void savePointCloud ();
        void loadPointCloud ();
        void togglePoints ();
        void toggleBalls ();
        void toggleDelaunayTriangulation ();
        void toggleVoronoiVertices ();
        void toggleVoronoiEdges ();
        void randomPointsEllipse ();
        void oneStep ();
        void computeGradients ();
        void toggleGradients ();
        void toggleDecomposition ();

    private:
        View *m_view;
        QWidget *m_rightside;

        QPushButton* m_resetButton;
        QPushButton* m_saveButton;
        QPushButton* m_loadButton;
        QPushButton* m_pointsButton;
        QPushButton* m_ballsButton;
        QPushButton* m_delaunayButton;
        QPushButton* m_voronoiVerticesButton, *m_voronoiEdgesButton;
        QPushButton* m_randomEllipseButton;
        QPushButton* m_oneStepButton;
        QPushButton *m_computeGradientsButton;
        QPushButton* m_gradientsButton;
        QPushButton* m_decompositionButton;
};

#endif

