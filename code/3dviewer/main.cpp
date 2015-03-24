#include "window.h"

#include <QApplication>

int main(int argc, char **argv) {
    srand(0);

    QApplication app(argc, argv);
    app.setApplicationName("3D Viewer");

    MainWindow mainWindow;
    mainWindow.show();

    return app.exec();
}

