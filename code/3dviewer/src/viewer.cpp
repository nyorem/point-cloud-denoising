#include "viewer.h"
#include "scene.h"

#include <QMouseEvent>
#include <QApplication>

Viewer::Viewer (QWidget* parent) :
    QGLViewer(parent),
    m_pScene(NULL),
    m_custom_mouse(false) {
    /* setBackgroundColor(::Qt::white); */
    toggleGridIsDrawn();
}

void Viewer::setScene (Scene* pScene) {
    m_pScene = pScene;
}

void Viewer::draw () {
    QGLViewer::draw();

    if (m_pScene != NULL) {
        ::glClearColor(1.0f,1.0f,1.0f,0.0f);
        m_pScene->render();
    }
}

void Viewer::initializeGL () {
    QGLViewer::initializeGL();
}

void Viewer::mousePressEvent (QMouseEvent* e) {
    if (e->modifiers() == Qt::ControlModifier) {
        m_custom_mouse = true;
    }

    QGLViewer::mousePressEvent(e);
}

void Viewer::mouseReleaseEvent (QMouseEvent* e) {
    if (m_custom_mouse) {
        QApplication::setOverrideCursor(Qt::WaitCursor);
        QApplication::restoreOverrideCursor();
        m_custom_mouse = false;
    }

    QGLViewer::mouseReleaseEvent(e);
}

