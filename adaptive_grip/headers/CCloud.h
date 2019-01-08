#ifndef CCLOUD__H
#define CCLOUD__H

#include "Mutex.h"
#include "ClusterFinder.h"

#include <pcl/point_cloud.h>

class CCloud {
public:

      typedef pcl::PointXYZRGBA        Point;
      typedef pcl::PointCloud<Point>   PointCloud;

      PointCloud::Ptr      cloud;
      bool                 initialized;
      PointCloud::Ptr      clusters;

      CCloud() {
         cf = NULL;
         cloud.reset(new PointCloud());
         initialized    = false;
      }

      CCloud(PointCloud::Ptr _cloud) {
         cf = NULL;
         cloud          = _cloud;
         initialized    = false;
      }

      int size() {
         return initialized ? clusters->size() : -1;
      }

      inline PointCloud::iterator begin() // convenience
      {
      // return clusters->points.begin();
			return clusters->begin();
      }

      inline PointCloud::iterator end() // convenience
      {
      // return clusters->points.end();
			return clusters->end();
      }

         ClusterFinder<Point> * cf;
      void find_clusters()
      {
         Mutex m; // TODO un-mutex this code

         if (cf != NULL) {
            delete cf;
         } 

         cf = new ClusterFinder<Point>(cloud, &m);

         cf->find_clusters();
         clusters = cf->get_clusters();
      }
};

#endif // CCloud.h
