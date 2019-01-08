#ifndef PCLUTILS__H
#define PCLUTILS__H

#include <pcl/common/geometry.h>
#include <cmath>
#include <iostream>

#include <pcl/point_representation.h>
#include <pcl/point_cloud.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Dense>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include "Logger.h"

#define DMSG(x) PCLUtils::dbWaitForEnter(x);

class MyPointRepresentation : public pcl::PointRepresentation <pcl::PointNormal>
{
  using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature std::vector
  virtual void copyToFloatArray (const pcl::PointNormal &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

namespace PCLUtils {

   typedef enum {
      X,
      Y,
      Z,
      J4,
      J5,
      J6
   } Direction;

   template <typename PointT>
   float vector_length(PointT point)
   {
      return sqrt((point.x * point.x) + (point.y * point.y) + (point.z * point.z));
   }

template <typename PointT>
void pairAlign (const typename pcl::PointCloud<PointT>::Ptr cloud_src, 
      const typename pcl::PointCloud<PointT>::Ptr cloud_tgt, 
      typename pcl::PointCloud<PointT>::Ptr output, 
      Eigen::Matrix4f &final_transform, 
      Logger * logger);

template <typename PointT>
   float angleBetween(const PointT &, const PointT &);

template <typename PointT>
   void project(const PointT &, const PointT &, PointT &);

template <typename PointT>
float dot_double_normalize(const PointT & original, const PointT & target);

template <typename PointT>
void subtractXYZ(const PointT&, const PointT&, PointT &);

template <typename PointT>
void addXYZ(const PointT&, const PointT&, PointT &);

template <typename PointT>
void multiplyXYZ(const PointT&, float, PointT &);

template <typename PointT>
float distance(const PointT&, const PointT&);

template <typename PointT>
void downsample(typename pcl::PointCloud<PointT>::Ptr, float);

void dbWaitForEnter(std::string);

}

#endif // PCLUTILS__H
