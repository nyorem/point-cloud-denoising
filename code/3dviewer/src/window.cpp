#include "window.h"
#include "viewer.h"
#include "scene.h"

#include "ui_polyhedron.h"

#include <CGAL/Qt/debug.h>

#include <QDragEnterEvent>
#include <QDropEvent>
#include <QTextStream>
#include <QUrl>
#include <QFileDialog>
#include <QSettings>
#include <QHeaderView>
#include <QClipboard>

MainWindow::MainWindow (QWidget* parent)
: CGAL::Qt::DemosMainWindow(parent) {
    ui = new Ui::MainWindow;
    ui->setupUi(this);

    // saves some pointers from ui, for latter use.
    m_pViewer = ui->viewer;

    // does not save the state of the viewer
    m_pViewer->setStateFileName(QString::null);

    // accepts drop events
    setAcceptDrops(true);

    // setups scene
    m_pScene = new Scene;
    m_pViewer->setScene(m_pScene);

    // connects actionQuit (Ctrl+Q) and qApp->quit()
    connect(ui->actionQuit, SIGNAL(triggered()), this, SLOT(quit()));

    this->addRecentFiles(ui->menuFile, ui->actionQuit);

    connect(this, SIGNAL(openRecentFile(QString)),
            this, SLOT(open(QString)));

    readSettings();
}

MainWindow::~MainWindow () {
    delete m_pScene;
    delete ui;
}

void MainWindow::dragEnterEvent (QDragEnterEvent *event) {
    if (event->mimeData()->hasFormat("text/uri-list"))
        event->acceptProposedAction();
}

void MainWindow::dropEvent (QDropEvent *event) {
    std::cout << "drop...";

    Q_FOREACH(QUrl url, event->mimeData()->urls()) {
        QString filename = url.toLocalFile();

        if(!filename.isEmpty()) {
            QTextStream(stderr) << QString("dropEvent(\"%1\")\n").arg(filename);
            open(filename);
        }
    }
    event->acceptProposedAction();
}

void MainWindow::closeEvent (QCloseEvent *event) {
    writeSettings();
    event->accept();
}

void MainWindow::quit() {
    writeSettings();
    close();
}

void MainWindow::readSettings () {
    this->readState("MainWindow", Size|State);
}

void MainWindow::writeSettings () {
    this->writeState("MainWindow");

    std::cerr << "Write setting...done" << std::endl;
}

void MainWindow::updateViewerBBox () {
    const Bbox bbox(-2.0, -2.0, -2.0, 2.0,  2.0,  2.0);

    const double xmin = bbox.xmin();
    const double ymin = bbox.ymin();
    const double zmin = bbox.zmin();
    const double xmax = bbox.xmax();
    const double ymax = bbox.ymax();
    const double zmax = bbox.zmax();

    qglviewer::Vec vec_min(xmin, ymin, zmin);
    qglviewer::Vec vec_max(xmax, ymax, zmax);

    m_pViewer->setSceneBoundingBox(vec_min, vec_max);
    m_pViewer->camera()->showEntireScene();
}

void MainWindow::on_actionView_ball_toggled () {
    m_pScene->toggle_view_ball();
    m_pViewer->update();
}

void MainWindow::on_actionView_edges_toggled () {
    m_pScene->toggle_view_edges();
    m_pViewer->update();
}

void MainWindow::on_actionView_facets_toggled () {
    m_pScene->toggle_view_facets();
    m_pViewer->update();
}

void MainWindow::on_actionView_smooth_toggled () {
    m_pScene->toggle_view_smooth();
    m_pViewer->update();
}

void MainWindow::on_actionView_clouds_toggled () {
    m_pScene->toggle_view_pointcloud();
    m_pViewer->update();
}

void MainWindow::on_actionView_field_toggled () {
    m_pScene->toggle_view_vectorfield();
    m_pViewer->update();
}

void MainWindow::open (QString filename) {
    QFileInfo fileinfo(filename);

    if (fileinfo.isFile() && fileinfo.isReadable()) {
        int index = m_pScene->open(filename);

        if (index >= 0) {
            QSettings settings;
            settings.setValue("open directory",
                              fileinfo.absoluteDir().absolutePath());
            this->addToRecentFiles(filename);

            // update bbox
            updateViewerBBox();
            m_pViewer->update();
        }
    }
}

void MainWindow::on_actionOpen_triggered () {
    QSettings settings;
    QString directory = settings.value("open directory",
                                       QDir::current().dirName()).toString();
    QStringList filenames =
        QFileDialog::getOpenFileNames(this,
                                      tr("Open polyhedral surface or point cloud..."),
                                      directory,
                                      tr("OFF files (*.off)\n"
                                         "XYZ files (*.xyz)\n"
                                         "All files (*)"));
    if (!filenames.isEmpty()) {
        Q_FOREACH (QString filename, filenames) {
            this->open(filename);
        }
    }
}

void MainWindow::on_actionReset_triggered () {
    if(m_pScene->is_visible_ball()) {
        m_pScene->toggle_view_ball();
    }
    m_pScene->clear_ball();

    if (m_pScene->is_visible_edges()) {
        m_pScene->toggle_view_edges();
    }
    if (m_pScene->is_visible_facets()) {
        m_pScene->toggle_view_facets();
    }
    m_pScene->clear_balls();

    if (m_pScene->is_visible_pointcloud()) {
        m_pScene->toggle_view_pointcloud();
    }
    m_pScene->clear_pointcloud();

    if (m_pScene->is_visible_vectorfield()) {
        m_pScene->toggle_view_vectorfield();
    }
    m_pScene->clear_vectorfield();

    m_pViewer->update();
}

void MainWindow::on_actionVectorField_triggered () {
    m_pScene->vector_field();
    m_pViewer->update();
}

void MainWindow::on_actionNsteps_triggered () {
    m_pScene->nsteps();
    m_pViewer->update();
}

