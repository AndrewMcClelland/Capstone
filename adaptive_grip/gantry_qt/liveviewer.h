#ifndef LIVEVIEWER__H
#define LIVEVIEWER__H

#include <boost/thread/thread.hpp>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/common_headers.h>
#include "Mutex.h"
#include "inst/pcl_visualizer.h"
#include <QtWidgets/QWidget>
#include <QtCore/QTimer>
#include <QVTKWidget.h>

class LiveViewer : QObject {
   Q_OBJECT
public:
   typedef pcl::visualization::PCLVisualizer    Visualizer;
   typedef pcl::PointCloud<pcl::PointXYZRGBA>   Cloud;

   pcl::OpenNIGrabber *         interface;
   QVTKWidget * vtk;
   Mutex * mutex;
   Visualizer::Ptr         viewer;
   boost::function<void (const Cloud::ConstPtr &)> callback;
   Cloud::Ptr        kinect_cloud;

   bool first;
   int live_viewport;

// LiveViewer(pcl::OpenNIGrabber * _interface, QVTKWidget * _vtk);
   LiveViewer(QVTKWidget * _vtk, Visualizer::Ptr, int);

   Visualizer::Ptr      get_viewer();

   bool start(); 

   bool stop();

   void update_cam_callback(const Cloud::ConstPtr &cloud);

   QTimer * timer;
   bool copying;

public slots:

   void processFrameAndUpdate();
   
};

#endif // LIVEVIEWER__H
