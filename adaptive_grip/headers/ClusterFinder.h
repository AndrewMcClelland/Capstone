#ifndef CLUSTERFINDER__H
#define CLUSTERFINDER__H

/*
   Name        :  ClusterFinder.h
   Author      :  Aaron
   Purpose     :  A class used to find Euclidean clusters within a point cloud.
                  The typename PointT specified the type of points in the point
                  cloud that a PlaneFilter object is working with.
                  
                  Note that this is a templated class so functions have to be 
                  implemented where they are declared.
*/

#include <iostream>
#include "common_defs.h"
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>

#include "Mutex.h"

#define CF_DEFAULT_VOXELSIZE 0.03f, 0.03f, 0.03f

// AN Feb. 12: CF_DEFAULT_TOLERANCE 0.10 was being used up to now.
// Small cluster tolerance - an actual object can be seen as multiple clusters.
// High cluster tolerance - multiple objects are seen as one cluster.
//#define CF_DEFAULT_TOLERANCE 0.10 // 10cm
#define CF_DEFAULT_TOLERANCE 0.02

#define CF_DEFAULT_MIN_CLUSTERSIZE 100
#define CF_DEFAULT_MAX_CLUSTERSIZE 25000

template <typename PointT> class ClusterFinder {
   public:

   typedef typename pcl::PointCloud<PointT>           Cloud;
   typedef typename pcl::PointCloud<PointT>::Ptr      CloudPtr;
   typedef typename pcl::PointCloud<PointT>::ConstPtr CloudConstPtr;

   // Constructor
   ClusterFinder(CloudPtr cloud, Mutex * mutex);


   // A vector of vectors.
   // Each element of m_clusters is a pcl::PointIndices with all the indices of m_cloud
   // corresponding to a specific cluster.
   std::vector<pcl::PointIndices>                     m_clusters;

   std::vector<PointT>                                m_centroids;
   
/*
   Function       : find_clusters()
   Description    : Try to identify clusters within a point clud.
   Returns        : The number of clusters identified, if successful.
                    A negative nubmer, if unsuccessful.
*/
   int find_clusters();
   
   void  setInputCloud(CloudPtr cloud);

   CloudPtr get_clusters();

   CloudPtr                                           _input_cloud;

   private:

   int _downsample();

   CloudPtr                                           _ds_cloud; // The downsampled point-cloud
   int                                                _downsample_factor;
   Mutex *                                            _cloud_mutex;

   pcl::EuclideanClusterExtraction<PointT>            _ece;
   typename pcl::search::KdTree<PointT>::Ptr          _tree;


};

#endif // CLUSTERFINDER__H
