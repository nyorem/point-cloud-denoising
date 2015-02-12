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

    public slots:
        void resetScene ();
        void togglePoints ();
        void toggleBalls ();
        void toggleDelaunayTriangulation ();
        void toggleVoronoiVertices ();
        void randomPointsEllipse ();
        void oneStep ();

    private:
        View *m_view;
        QWidget *m_rightside;

        QPushButton* m_resetButton;
        QPushButton* m_pointsButton;
        QPushButton* m_ballsButton;
        QPushButton* m_delaunayButton;
        QPushButton* m_voronoiVerticesButton;
        QPushButton* m_randomEllipseButton;
        QPushButton* m_oneStepButton;
};

#endif

