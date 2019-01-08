#include "Engine.h"

Engine::Engine(PointCloud::Ptr      cloud,
               Mutex *              mutex,
               Logger *             log) :
   _cloud(cloud),
   _mutex(mutex),
   m_camera(new Camera<Point>(_cloud, _mutex)),
   m_plane_filter(new TwoPlaneFilter<Point>(_cloud, _mutex)),
   m_cluster_finder(_cloud, _mutex),
   m_logger(log)
{
   m_vis_cloud.reset(new PointCloud());
   m_state = GOT_NOTHING;
}

void
Engine::get_image() // TODO BROKEN 1/21/2017
{
   if (m_camera->retrieve() == SUCCESS) {
      m_state = GOT_CLOUD;
   } else {
      m_logger->log("Could not retrieve an image!");
      m_state = GOT_NOTHING;
   };

   // Copy the cloud
   // VISUALIZATION
   m_vis_cloud.reset(new PointCloud());
   *m_vis_cloud = *_cloud;
}

void
Engine::get_floor()
{
// m_logger->log("Engine::get_floor() not yet implemented!");
   
// m_plane_filter->get_filtered_centroid(claw_point);
   m_plane_filter->filter_plane();

   // FYI : http://www.cprogramming.com/tutorial/references.html
// const std::vector<int> & indices = m_plane_filter->get_inliers();

// // TODO should export these to a different cloud.
// // Colour them
// for (std::vector<int>::const_iterator it = indices.begin();
//       it != indices.end();
//       ++it)
// {
//    m_vis_cloud->points[*it].r = 0xFF;
//    m_vis_cloud->points[*it].g = 0xB6;
//    m_vis_cloud->points[*it].b = 0xC1;
//    m_vis_cloud->points[*it].a = _floor_alpha_value; // make them invisible
// }

   m_vis_centroids.reset(new PointCloud);

   Point clawPoint = m_plane_filter->claw_point;

   clawPoint.r = 0;
   clawPoint.g = 255;
   clawPoint.b = 0;
   clawPoint.a = 255;

   m_vis_centroids->push_back(clawPoint);

   colour_floor(128);

   m_floor_plane[0] = m_plane_filter->m_plane->values[0];
   m_floor_plane[1] = m_plane_filter->m_plane->values[1];
   m_floor_plane[2] = m_plane_filter->m_plane->values[2];
   m_floor_plane[3] = m_plane_filter->m_plane->values[3];
   
}

void 
Engine::colour_floor(int alpha_val)
{
   const std::vector<int> & indices = m_plane_filter->get_inliers();
   for (std::vector<int>::const_iterator it = indices.begin();
         it != indices.end();
         ++it)
   {
      m_vis_cloud->points[*it].r = 0xFF;
      m_vis_cloud->points[*it].g = 0xB6;
      m_vis_cloud->points[*it].b = 0xC1;
      m_vis_cloud->points[*it].a = alpha_val; // make them invisible
   }
}

void
Engine::get_clusters()
{
// m_logger->log("Engine::get_clusters() not yet implemented!");

   const int clusters = m_cluster_finder.find_clusters();

   // This is how we estimate where the claw is
   
   // VISUALIZATION -- create points corresponding to the clusters
// m_vis_centroids.reset(new PointCloud());
   

   std::stringstream s("");
   s << "Found " << clusters << " clusters.";
   m_logger->log(s.str());


   for (std::vector<Point>::const_iterator it = m_cluster_finder.m_centroids.begin();
         it != m_cluster_finder.m_centroids.end();
         ++it)
   {
      Point    p = *it;

    //p.r = colours::colours[colours::PURPLE].red;
    //p.g = colours::colours[colours::PURPLE].green;
      p.r = 255;
      p.g = 0;
      p.b = 0;
      p.a = 255;

      m_vis_centroids->push_back(p);
   }

   // 1. Get Centroids in a Point Cloud
   PointCloud::Ptr centroidProjectionsCloud(new PointCloud());
   PointCloud::Ptr centroidCloud = PointCloud::Ptr(new PointCloud());

   for (std::vector<Point>::iterator it = m_cluster_finder.m_centroids.begin();
       it != m_cluster_finder.m_centroids.end();
       it++)
   {
      Point p = (*it);
      centroidCloud->points.push_back(p);
   }

   if (clusters == 0) {
      m_logger->log("No clusters were found. Returning.");
      return;
   }

   Eigen::Vector4f      tmp_plane = m_plane_filter->get_plane_coefficients();

   // Find the highest cluster. Assign it to m_claw_cluster.
   std::vector<Point>::iterator highest; // Theoretically safer to make it equal to m_centroids.begin()
   float                        longest_distance = 0.0;
   for (std::vector<Point>::iterator it = m_cluster_finder.m_centroids.begin(); 
         it != m_cluster_finder.m_centroids.end(); ++it) {
         Cluster<Point> c(*it);
         float tmp = c.get_distance_to_plane(tmp_plane);
         std::stringstream s;
         s << "Cluster found with distance: " << tmp << std::endl;
         m_logger->log(s);
         if ( fabs(tmp) > longest_distance ) {
            s.clear();
            s << "Longest Distance: " << longest_distance << std::endl;
            longest_distance = tmp;
            highest = it;
         }
   }

   // Remove `highest` iterator
   m_claw_cluster = *highest;
   m_cluster_finder.m_centroids.erase(highest);

  // http://docs.pointclouds.org/1.7.1/sac__model__plane_8hpp_source.html 
  // Something's wrong in project_points... TODO investigate OR use this workaround

   // Clear the engine's list of clusters
   m_clusters.clear();
   for (PointCloud::iterator it = centroidCloud->begin();
         it != centroidCloud->end();
         it++)
   {
      Cluster<Point> c(*(it));
      m_clusters.push_back(c);
   }
}

void
Engine::get_movement()
{
   // Steps:
   // Project the "viewer's" x-axis on to the floor plane. Let this be the 
   // temporary "x-axis".
   //
   // Rotate it by 90 degrees to get a temporary "y-axis"
   m_logger->log("Engine::get_movement() not yet implemented!");
}

