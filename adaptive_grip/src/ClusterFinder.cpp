
#include "ClusterFinder.h"

template <typename PointT>
ClusterFinder<PointT>::ClusterFinder(CloudPtr cloud, Mutex * mutex) :
   _input_cloud(cloud),
   _cloud_mutex(mutex),
   _tree(new pcl::search::KdTree<PointT>),
   _ds_cloud(CloudPtr(new Cloud))
{
   // Euclidean cluster extraction settings
   _ece.setClusterTolerance(CF_DEFAULT_TOLERANCE);
   _ece.setMinClusterSize(CF_DEFAULT_MIN_CLUSTERSIZE);
   _ece.setMaxClusterSize(CF_DEFAULT_MAX_CLUSTERSIZE);
   _ece.setSearchMethod(_tree);
   _ece.setInputCloud(_input_cloud);
}

template <typename PointT>
int
ClusterFinder<PointT>::find_clusters()
{
   std::vector<int> nan_indices;
   pcl::removeNaNFromPointCloud(*_input_cloud, *_input_cloud, nan_indices);
   // Downsample
   _downsample();

   // Construct the 3D-Tree
   _tree->setInputCloud(_input_cloud);

   // Extract
   _ece.extract(m_clusters);

   if (m_clusters.size() > 0) {

      // populate m_centroids.
      m_centroids.clear();

      for (std::vector<pcl::PointIndices>::const_iterator cluster_it = m_clusters.begin();
           cluster_it != m_clusters.end(); ++cluster_it)
      {
         pcl::CentroidPoint<PointT>   centroid_calc; // create a new centroid
         const std::vector<int> &      indices = (*cluster_it).indices;

         for (std::vector<int>::const_iterator it = indices.begin();
              it != indices.end(); ++it)
         {
            centroid_calc.add((*_input_cloud)[*it]);
         }

         PointT    centroid;
         centroid_calc.get(centroid);
         m_centroids.push_back(centroid);
      }
      
      return m_clusters.size();
   } else {
      return -1;
   }
}

template <typename PointT>
int
ClusterFinder<PointT>::_downsample()
{
   // TODO
   return SUCCESS;
}

template <typename PointT>
void
ClusterFinder<PointT>::setInputCloud(CloudPtr cloud)
{
   _input_cloud = cloud;
}

template <typename PointT>
typename ClusterFinder<PointT>::CloudPtr
ClusterFinder<PointT>::get_clusters()
{
   CloudPtr   clusterCloud(new Cloud);
   for (typename std::vector<PointT>::const_iterator it = m_centroids.begin();
         it != m_centroids.end(); ++it)
   {
      clusterCloud->push_back(*it);
   }
   return clusterCloud;
}

template class ClusterFinder<pcl::PointXYZ>;
template class ClusterFinder<pcl::PointXYZRGBA>;

