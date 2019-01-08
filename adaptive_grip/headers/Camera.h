#ifndef CAMERA__H
#define CAMERA__H

#include <boost/thread/thread.hpp>
#include <pcl/io/openni_grabber.h> 
#include <pcl/common/common_headers.h>
#include "common_defs.h"
#include "Mutex.h"
#include "CameraBase.h"


template <typename PointT> class Camera : public CameraBase<PointT> { 

   typedef typename pcl::PointCloud<PointT>              Cloud;
   typedef typename pcl::PointCloud<PointT>::Ptr         CloudPtr;
   typedef typename pcl::PointCloud<PointT>::ConstPtr   CloudConstPtr;

public:

// CloudPtr    m_cloud;

   // Constructor
   Camera(CloudPtr cloud, Mutex * mutex);

   // 0 if good, non-zero if bad
   int   retrieve();
   



private:

   void                             _update_cloud_callback(const typename pcl::PointCloud<PointT>::ConstPtr &cloud); 
   CloudConstPtr                    _camera_cloud;
   Mutex *                          _cloud_mutex;
   bool                             _retrieved_cloud;
   pcl::Grabber *                   _interface;

};

#endif // CAMERA__H
