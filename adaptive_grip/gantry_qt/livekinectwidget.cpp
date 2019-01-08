#include "livekinectwidget.h"

LiveKinectWidget::LiveKinectWidget(QVTKWidget * _vtk) :
   vtk(_vtk)
{

   boost::function<void (const Cloud::ConstPtr &)> cam_callback =
      boost::bind(&LiveKinectWidget::_live_camera_callback, this, _1);

   interface = new pcl::OpenNIGrabber();
   interface->registerCallback(cam_callback);
// interface->start();

   // Configure the viewer

}

bool LiveKinectWidget::start()
{
   // Try it without new threads, see what happens
   interface->start();
   return true;
}

bool LiveKinectWidget::stop()
{
   // Try it without new threads, see what happens
   interface->stop();
   return true;
}

void LiveKinectWidget::_live_camera_callback(const Cloud::ConstPtr &cloud)
{
   mutex->lock();
   live_visualizer->updatePointCloud(cloud, "live_cloud");
   mutex->unlock();
}
