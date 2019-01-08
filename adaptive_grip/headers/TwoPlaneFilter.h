#ifndef TWOPLANEFILTER__H
#define TWOPLANEFILTER__H 

/*
   Name        :  TwoPlaneFilter.h
   Author      :  Aaron
   Purpose     :  An extension of PlaneFilter.h that filters out all points
                  above one plane, and below a second.
                  The two planes are parallel and are separated by the distance
                  contained in `m_plane_separation`
*/

#include "PlaneFilter.h"
#include <pcl/common/centroid.h>


template <typename PointT> class TwoPlaneFilter : public PlaneFilter<PointT> {

   using PlaneFilter<PointT>::_inliers;
   using PlaneFilter<PointT>::_clipper;
   using PlaneFilter<PointT>::_cloud;

   public:

   using PlaneFilter<PointT>::m_plane;

      typedef typename pcl::PointCloud<PointT>           Cloud;
      typedef typename pcl::PointCloud<PointT>::Ptr      CloudPtr;
      typedef typename pcl::PointCloud<PointT>::ConstPtr CloudConstPtr;

      double                           m_plane_separation;
      pcl::ModelCoefficients::Ptr      m_plane_two;


      TwoPlaneFilter(CloudPtr cloud, Mutex * mutex, double plane_separation = DEFAULT_PLANE_SEPARATION);


      // Exclusive Function
      void get_filtered_centroid(PointT & output);

   private:

      pcl::ExtractIndices<PointT>      _extract_two;
      Eigen::Vector4f                  _plane_two_vector;

      virtual void _compute_separator();
      

      virtual void _filter_cloud();
      

};

#endif // TWOPLANEFILTER__H
