#include "CameraFile.h"

template <typename PointT>
CameraFile<PointT>::CameraFile(CloudPtr cloud, Mutex * mutex) :
   CameraBase<PointT>(cloud),
   _cloud_mutex(mutex),
   filename("/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/data/registration/ball_one.pcd")
{}

template <typename PointT>
int
CameraFile<PointT>::retrieve()
{      
   _cloud_mutex->lock();

   if (pcl::io::loadPCDFile<PointT>(filename, *(this->m_cloud)) == -1) {
      std::cerr << "Could not open: <" << filename << ">!" << std::endl;
      _cloud_mutex->unlock();
      return FAILURE;
   }

   //    Don't need this any more - use a ColorHandler
//    for (typename pcl::PointCloud<PointT>::iterator it = this->m_cloud->begin();
//         it != this->m_cloud->end(); 
//         ++it)
//    {
//       (*it).r = 255;
//       (*it).g = 255;
//       (*it).b = 255;
//    }

   _cloud_mutex->unlock();
   return SUCCESS;
}

template <typename PointT>
void
CameraFile<PointT>::setFilename(std::string s)
{
   filename = s;
}

template class CameraFile<pcl::PointXYZ>;
template class CameraFile<pcl::PointXYZRGBA>;

