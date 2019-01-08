
#include "PlaneFilter.h"

template<typename PointT> 
int
PlaneFilter<PointT>::filter_plane()
{
   if (_downsample_factor > 0) {
      // TODO optionally downsample...
   }

   _seg.segment(*_inliers, *m_plane); 

   // TODO this check could be better.. maybe check if 15% of the points
   // in the point cloud were removed?
   if (_inliers->indices.size() == 0) {
      std::cerr << "PlaneFilter::filter_plane() - no plane inliers!" << std::endl;
      return FAILURE;
   } 


   else {
      // Acquire the cloud's mutex while we're in this scope
      Mutex::ScopedLock(*_cloud_mutex);

      _compute_separator();
      
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*_cloud, *_cloud, indices);

      get_filtered_centroid(claw_point);
      _filter_cloud();

      return SUCCESS;
   }
}

template<typename PointT> 
const std::vector<int> &
PlaneFilter<PointT>::get_inliers() const
{
   return _inliers->indices;
}


template<typename PointT> 
Eigen::Vector4f
PlaneFilter<PointT>::get_plane_coefficients() 
{
   return _plane_vector;
}

template<typename PointT> 
void
PlaneFilter<PointT>::_compute_separator() 
{
   _plane_vector << m_plane->values[0], m_plane->values[1], m_plane->values[2], 
                (m_plane->values[3] + DEFAULT_FLOOR_HEIGHT);
}


template<typename PointT> 
void
PlaneFilter<PointT>::_filter_cloud() 
{
   std::cout << "DEBUG: before filter, _cloud->size() = " << _cloud->size() << std::endl;

   _clipper.setPlaneParameters(_plane_vector);
   _inliers->indices.clear();
   _clipper.clipPointCloud3D(*_cloud, _inliers->indices);
   _extract.setIndices(_inliers);
   _extract.filterDirectly(_cloud);

   std::cout << "DEBUG: after filter, _cloud->size() = " << _cloud->size() << std::endl;
}

template<typename PointT> 
PlaneFilter<PointT>::PlaneFilter(CloudPtr cloud, Mutex * mutex):
   _cloud(cloud),
   _cloud_mutex(mutex),
   m_plane(new pcl::ModelCoefficients()),
   _floor_extract_height(DEFAULT_FLOOR_HEIGHT),
   _clipper(_plane_vector),
   _inliers(new pcl::PointIndices())
{
   _downsample_factor = -1;

   // default settings..
   _seg.setOptimizeCoefficients(true);
   _seg.setModelType(pcl::SACMODEL_PLANE);
   _seg.setMethodType(pcl::SAC_RANSAC);
   _seg.setMaxIterations(1000);
   _seg.setDistanceThreshold(0.01);
   _seg.setInputCloud(_cloud);

   // extractor settings..
//    _extract.setIndices(_inliers->indices);
   _extract.setNegative(true);
}

template <typename PointT> void
PlaneFilter<PointT>::get_filtered_centroid(PointT & output)
{
   // TODO 
   std::cerr << "PlaneFilter::get_filtered_centroid() has not been implemented!" << std::endl;
}

template <typename PointT> void
PlaneFilter<PointT>::set_input_cloud(CloudPtr input_cloud)
{
   _cloud = input_cloud;
}

template class PlaneFilter<pcl::PointXYZ>;
template class PlaneFilter<pcl::PointXYZRGBA>;
