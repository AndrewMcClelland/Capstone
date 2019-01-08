#include "TwoPlaneFilter.h"


template <typename PointT> 
TwoPlaneFilter<PointT>::TwoPlaneFilter(CloudPtr cloud, Mutex * mutex, double plane_separation) :
   PlaneFilter<PointT>(cloud, mutex),
   m_plane_separation(plane_separation),
   m_plane_two(new pcl::ModelCoefficients())
{
   _extract_two.setNegative(false);
}

template <typename PointT>
void
TwoPlaneFilter<PointT>::_compute_separator()
{
      PlaneFilter<PointT>::_compute_separator();

      _plane_two_vector << m_plane->values[0], m_plane->values[1], m_plane->values[2], 
                   (m_plane->values[3] + DEFAULT_FLOOR_HEIGHT + m_plane_separation);
}

template <typename PointT> void TwoPlaneFilter<PointT>::_filter_cloud()
{
   PlaneFilter<PointT>::_filter_cloud();

   _clipper.setPlaneParameters(_plane_two_vector);
   _inliers->indices.clear();
   _clipper.clipPointCloud3D(*_cloud, _inliers->indices);

   _extract_two.setIndices(_inliers);
   _extract_two.filterDirectly(_cloud);

}


template <typename PointT> void TwoPlaneFilter<PointT>::get_filtered_centroid(PointT & output)
{
   // Get all of the indices above the second plane
   _compute_separator();

// pcl::PlaneClipper3D<PointT>      clipper(_plane_two_vector);
// pcl::PointIndices::Ptr                below_plane_indices(new pcl::PointIndices());
// std::vector<int>                above_plane_indices;

// clipper.clipPointCloud3D(*_cloud, below_plane_indices->indices);

// pcl::ExtractIndices<PointT>              invert;
// invert.setInputCloud(_cloud);
// invert.setIndices(below_plane_indices);
// invert.setNegative(true);
// invert.filter(above_plane_indices);

// std::cout << "get_filtered_centroid(): Adding " << above_plane_indices.size() << " points to centroid calculator." << std::endl;
// // Then, go and compute centroid
// pcl::CentroidPoint<PointT>       centroid_calc;
// 
// for (std::vector<int>::const_iterator it = above_plane_indices.begin();
//       it != above_plane_indices.end(); ++it)
// {
//    centroid_calc.add((*_cloud)[*it]);
// }

// centroid_calc.get(output);
}
 



template class TwoPlaneFilter<pcl::PointXYZ>;
template class TwoPlaneFilter<pcl::PointXYZRGBA>;
