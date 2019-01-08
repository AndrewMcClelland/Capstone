#ifndef PCLVIEWER2_H
#define PCLVIEWER2_H

#include <QMainWindow>

#include <iostream>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// VTK
#include <vtkRenderWindow.h>

#include "Mutex.h"
#include "CameraBase.h"
// #include "Logger.h"

#include <QString.h> // duct tape
#include <QTextEdit> 

#include "QLogger.h"
#include "Engine.h"

// TODO move this somewhere better
// can't do Q_ASSERT(CONNECT( ... )) b/c assert expands to a no-op in release builds
#define CHECKED_CONNECT(source, signal, receiver, slot) \
   if (!connect(source, signal, receiver, slot)) \
      qt_assert_x(Q_FUNC_INFO, "CHECKED_CONNECT failed", __FILE__, __LINE__);


namespace Ui {
class PCLViewer2;
}

class PCLViewer2 : public QMainWindow
{
    Q_OBJECT

    typedef pcl::PointXYZRGBA       Point;
    typedef pcl::PointCloud<Point>  PointCloud;

public:
    explicit PCLViewer2(QWidget *parent = 0);
    ~PCLViewer2();

protected:
    pcl::visualization::PCLVisualizer::Ptr      viewer;
    PointCloud::Ptr                             cloud;
    Mutex * 								            mutex;
    CameraBase<Point> *                         camera;

public slots:
    void    capture_image();
    void    detect_floor();
    void    find_clusters();
    void    move_claw();

//  void    adjust_floor_alpha(int);

private:
    Ui::PCLViewer2 *ui;
    void _update_cloud();

    Engine *  _engine;
    Logger *                                    logger;
};

#endif // PCLVIEWER2_H
