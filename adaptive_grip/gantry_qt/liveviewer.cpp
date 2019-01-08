#include "liveviewer.h"
using pcl::OpenNIGrabber;

// LiveViewer::LiveViewer(OpenNIGrabber * _interface, QVTKWidget * _vtk) {
   LiveViewer::LiveViewer(QVTKWidget * _vtk, LiveViewer::Visualizer::Ptr _viewer, int viewport) {

//    viewer.reset(new Visualizer("LiveViewer", false));
      callback = boost::bind(&LiveViewer::update_cam_callback, this, _1);
      mutex = new Mutex();
      kinect_cloud.reset(new Cloud());
      vtk = _vtk;

      viewer = _viewer;
      live_viewport = viewport;

      interface = NULL;

      timer = new QTimer(this);
      connect(timer, SIGNAL(timeout()), this, SLOT(processFrameAndUpdate()));

      copying = false;
      first = true;
   }

   LiveViewer::Visualizer::Ptr      LiveViewer::get_viewer() {
      return viewer;
   }

   bool LiveViewer::start() {
      if (interface == NULL) {
         interface = new OpenNIGrabber();
      }
      interface->stop(); // Make Sure
      interface->registerCallback(callback);
      interface->start();
      timer->start(100);

   }

   bool LiveViewer::stop() {
      timer->stop();
      if (interface != NULL) {
         interface->stop();
//       delete interface;
//       interface = NULL;
      }
   }

   void LiveViewer::update_cam_callback(const Cloud::ConstPtr &cloud)
   {
      while (copying)
         sleep(0);
      copying = true;
      mutex->lock();
      pcl::copyPointCloud(*cloud, *kinect_cloud);
   // viewer->updatePointCloud(cloud, "liveCloud");
      mutex->unlock();
      copying = false;
   }

void LiveViewer::processFrameAndUpdate()
{
   if (copying == false)
   {
      // update
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr kinectCloudCopy (new pcl::PointCloud<pcl::PointXYZRGBA>());
      pcl::copyPointCloud(*kinect_cloud, *kinectCloudCopy);
      if (first) {
         viewer->addPointCloud<pcl::PointXYZRGBA>(kinectCloudCopy, "liveCloud", live_viewport);
         first = false;
      }
      viewer->updatePointCloud(kinectCloudCopy, "liveCloud");
  //  vtk->GetInteractor()->Render();
      vtk->update();
   }
}
