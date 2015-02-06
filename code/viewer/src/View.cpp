#include "View.hpp"
#include "Scene.hpp"

#include <QApplication>
#include <QGraphicsView>
#include <QKeyEvent>
#include <QMouseEvent>

View::View (int w, int h, QWidget *parent) : QGraphicsView(parent) {
    resize(w, h);

    m_scene = new Scene();
    setScene(m_scene);
}

void View::mousePressEvent (QMouseEvent *event) {
    // Initial origin: center of the view
    // mapToScene transforms the coordinates accordingly
    QPointF point = mapToScene(event->pos());
    m_scene->addPoint(point.x(), point.y());
}

void View::keyPressEvent (QKeyEvent *event) {
    switch (event->key()) {
        case Qt::Key_Escape:
            qApp->quit();
            break;

        default:
            break;
    }
}

