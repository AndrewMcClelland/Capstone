#include "Camera.h"

#define DEBUG_CAMERA

template <typename PointT> 
Camera<PointT>::Camera(CloudPtr cloud, Mutex * mutex) :
   CameraBase<PointT>(cloud),
   _cloud_mutex(mutex)
{
   _retrieved_cloud = false;

   // Note to self - you need to pass in an address to a function for boost::bind
   // boost::bind(__function_name__ ...) will segfault and not tell you why
   // boost::bind(&__function_name__ ...) is the correct call
   boost::function<void (const typename pcl::PointCloud<PointT>::ConstPtr &)>   callback_function =
      boost::bind(&Camera::_update_cloud_callback, this, _1); 
   
   // Instantiate the interface and register the callback.
   _interface = new pcl::OpenNIGrabber();
   _interface->registerCallback(callback_function);
   _interface->stop();
}

template <typename PointT> 
int
Camera<PointT>::retrieve()
{
   // Acquire the cloud mutex before we modify the cloud.
#ifdef DEBUG_CAMERA
   std::cout << "Waiting for camera mutex.." << std::endl;
#endif

   // Force _retrieved_cloud to false just in case the callback over-fired
   _retrieved_cloud = false;

	// Start the interface
   _interface->start();

   // Wait until the Kinect has picked up a new cloud.
   do { 
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
   } while (!_retrieved_cloud);

   // Modify the cloud, and release the Mutex.
   _cloud_mutex->lock();
#ifdef DEBUG_CAMERA
   std::cout << "Acquired camera mutex.." << std::endl;
#endif
   *(this->m_cloud) = *_camera_cloud; // quirk with template inheritance, need to access superclass members with this->
   _interface->stop();
   _cloud_mutex->unlock();
#ifdef DEBUG_CAMERA
// std::cout << "Released camera mutex.." << std::endl;
#endif

   _retrieved_cloud = false;
   
   return SUCCESS;
}

template <typename PointT> 
void
Camera<PointT>::_update_cloud_callback(const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
{
   _cloud_mutex->lock();
   _camera_cloud = cloud; // Is this wrong? A: nope
   _retrieved_cloud = true;
#ifdef DEBUG_CAMERA
// std::cout << "Camera::Update." << std::endl;
#endif
   _cloud_mutex->unlock();
}


template class Camera<pcl::PointXYZ>;
template class Camera<pcl::PointXYZRGBA>;
