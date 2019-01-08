#ifndef PLANEFILTER__H
#define PLANEFILTER__H

/*
   Name        :  PlaneFilter.h
   Author      :  Aaron
   Purpose     :  A class used to "filter" all points in a given plane out of 
                  a point cloud.
                  For example, this class can be used to remove all points 
                  representing the floor from a point cloud captured from
                  the Gantry Robot's Kinect.

                  The typename PointT specified the type of points in the point
                  cloud that a PlaneFilter object is working with.
                  
                  Note that this is a templated class so functions have to be 
                  implemented where they are declared.
*/

#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/plane_clipper3D.h> // Clip plane
#include <inst/plane_clipper_3D.h> // Instantiations Version
#include "common_defs.h"
#include "Mutex.h"



template <typename PointT> class PlaneFilter {

public:

   // Convenience
   typedef typename pcl::PointCloud<PointT>           Cloud;
   typedef typename pcl::PointCloud<PointT>::Ptr      CloudPtr;
   typedef typename pcl::PointCloud<PointT>::ConstPtr CloudConstPtr;
   typedef typename pcl::PlaneClipper3D<PointT>       Clipper;

   // Coefficients of the plane that was detected.
   pcl::ModelCoefficients::Ptr      m_plane;

/*
   Function       : filter_plane()
   Description    : Tries to detect a plane in m_cloud. If successfull, removes
                    the points within `_floor_extract_height` of that plane.
                    `_floor_extract_height` is set to DEFAULT_FLOOR_HEIGHT 
                    by default.
   Returns        : SUCCESS, if a plane was detected and its corresponding 
                    points were removed.
                    FAILURE, if a plane was not detected.
*/
   int filter_plane(); 

/*
   Function       : get_inliers()
   Description    : Tries to detect a plane in m_cloud. If successfull, removes
                    the points within `_floor_extract_height` of that plane.
                    `_floor_extract_height` is set to DEFAULT_FLOOR_HEIGHT 
                    by default.
   Returns        : A reference to the std::vector<int> of indices that were
                    removed from the last point cloud.
*/
   const std::vector<int> & get_inliers() const;

   PlaneFilter(CloudPtr cloud, Mutex * mutex);

/*
   Function       : get_plane_coefficients()
   Description    : Returns a Vector4f (basically double[4]) representing the 
                    coefficients of the detected plane.
   Returns        : See description
*/
   Eigen::Vector4f                        get_plane_coefficients();

   virtual void get_filtered_centroid(PointT & output);

   PointT                                 claw_point;

   void set_input_cloud(CloudPtr input_cloud);
protected:

   CloudPtr                               _cloud;
   int                                    _downsample_factor;
   Mutex *                                _cloud_mutex;
   pcl::SACSegmentation<PointT>           _seg;
   pcl::PointIndices::Ptr                 _inliers;
   pcl::ExtractIndices<PointT>            _extract;
   Clipper                                _clipper;
   Eigen::Vector4f                        _plane_vector;
   double                                 _floor_extract_height;


   virtual void _compute_separator();

   virtual void _filter_cloud();

};

#endif
