#ifndef LIVEKINECTWIDGET_H
#define LIVEKINECTWIDGET_H

#include <boost/thread/thread.hpp>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/common_headers.h>
#include "Mutex.h"
#include "inst/pcl_visualizer.h"
#include <QWidget>

class LiveKinectWidget
{
public:

   typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
   typedef pcl::visualization::PCLVisualizer  Viewer;

   Mutex *                                   mutex;
   Viewer::Ptr                               viewer;
   pcl::Grabber *                            interface;
   QVTKWidget *                              vtk;

   bool start();
   bool stop();

   LiveKinectWidget(QVTKWidget *);

private:

    _live_camera_callback(const Cloud::ConstPtr &cloud);
};

#endif // LIVEKINECTWIDGET_H
