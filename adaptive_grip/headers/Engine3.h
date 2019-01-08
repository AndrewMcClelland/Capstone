#include "Engine2.h"
#include <Eigen/Dense>
#include <pcl/visualization/pcl_plotter.h>

#define PI 3.14159265

class Engine3 : public Engine2 {

public: 

   pcl::visualization::PCLPlotter * plotter;

   Engine3(
         pcl::visualization::PCLVisualizer::Ptr _vis,
         Logger * _logger) : Engine2(_vis, _logger) {
      plotter = NULL;   }

   PointCloud::Ptr closest_cluster_cloud;
   double          claw_angle;

   int scan()
   {
      Engine2::scan();

   // return get_approach();

   // closest_cluster_cloud.reset(new PointCloud());
   // 
   // // TODO this will break any of the "center" or "move to" methods
   // // Colour the closest cluster. Will be at the same index as the 'centroid'
   // const std::vector<WSObject>::iterator closest = get_closest_object();

   // // TODO THIS WILL ONLY BE TRUE FOR THE FIRST SCAN. THE INDEX IN CLUSTERS WILL
   // // BE DIFFERENT THAN THE INDEX IN "OBJECTS"
   // int cluster_index = std::distance(m_objects->begin(), closest);
   // {
   //    stringstream msg;
   //    msg << "Engine3(): current_view.cf->m_clusters.size() = " << current_view.cf->m_clusters.size() << endl;
   //    msg << "closest_index = " << cluster_index << endl;
   //    msg << "Engine3()::scan - size of closest cluster = " << current_view.cf->m_clusters[cluster_index].indices.size();
   //    std::cout  << msg.str() << endl;
   //    m_logger->log(msg);
   // }


   // typedef std::vector<int>::const_iterator IndicesIt;
   // for (IndicesIt it = current_view.cf->m_clusters[cluster_index].indices.begin();
   //       it != current_view.cf->m_clusters[cluster_index].indices.end(); ++it)
   // {
   //    // copy these indices of current_view
   //    closest_cluster_cloud->push_back((*current_view.cloud)[*it]);
   // }

   // add_cloud_to_viewer(closest_cluster_cloud, "ClosestCluster", vp_calibration_axes, 255, 0, 0);

   // // Now, we have the entire cluster, and its 
      
   // create_surface_map(closest_cluster_cloud, (*current_view.clusters)[cluster_index]);

   }

   int move_to_object(int index)
   {
      m_logger->log("Entering Engine3::move_to_object()");

      // TODO move this-------------------------------
      // Check if the index is valid
      if (index < 0 || index >= m_objects->size()) {
         std::stringstream emsg;
         emsg << "Engine3::move_to_object() - Illegal index (" << index << ") provided. m_objects->size() = "
            << m_objects->size() << ".";
         m_logger->log(emsg);
         return FAILURE;
      }
      std::vector<WSObject>::iterator object=m_objects->begin();
      std::advance(object, index);
      // TODO move this-------------------------------
      
      // Move to our vantage point
      vantage_point(object);

      float pickup_angle, object_height;

      get_approach(object, pickup_angle, object_height);

//    pickup(object);
   }

   int get_approach(std::vector<WSObject>::iterator object, float & angle, float & height)
   {
      typedef std::vector<WSObject>::iterator   ObjectIterator;
  //  typedef pcl::PointCloud<Point>::iterator  PointIterator;
      typedef std::vector<Point>::iterator  PointIterator;
      

      // Get the object we want before we re-scan.
//    ObjectIterator object =           get_closest_object();
      PointIterator  closest_object_centroid;
      const RobotPosition  currentPos = getPosition();

      // Re-scan.
      Engine2::scan();

      // Go through the clusters in the current view, and find the one that's closest
      // to the object we are trying to approach.
      float shortest_xy_dist = -1;
      RobotPosition calculatedPosition = currentPos;
      for (PointIterator point = current_view.cf->m_centroids.begin(); point != current_view.cf->m_centroids.end(); ++point)
      {
         calculatedPosition = currentPos;
         calculate_robot_position(*point, calculatedPosition);

         float x_delta = calculatedPosition.x - object->x_position;
         float y_delta = calculatedPosition.y - object->y_position;
         float distance = sqrt(x_delta * x_delta + y_delta * y_delta);

         if (shortest_xy_dist < 0 || distance < shortest_xy_dist)
         {
            shortest_xy_dist = distance;
            closest_object_centroid = point;
         }
      }

   // {
   //    std::stringstream msg;
   //    msg << "The WSObject is at (" << object->x_position << ", " << object->y_position << endl;
   //    msg << "The Closest Cluster is at (" << calculatedPosition.x << ", " << calculatedPosition.y;
   //    m_logger->log(msg);
   // }

      // Now, get the index of the cloud in the "CCloud" current_view
      int cluster_index = std::distance(current_view.cf->m_centroids.begin(), closest_object_centroid);

   // stringstream msg;
   // msg << "get_approach(): cluster_index = " << cluster_index << endl;
   // msg << "current_view.clusters->size() = " << current_view.clusters->size() << endl;
   // msg << "current_view.cf->m_clusters.size() = " << current_view.cf->m_clusters.size();
   // msg << "current_view.cf->m_centroids.size() = " << current_view.cf->m_centroids.size();
   // m_logger->log(msg);

      typedef std::vector<int>::const_iterator IndicesIt;
      std::vector<int> & points_vector = current_view.cf->m_clusters[cluster_index].indices; // save typing

      closest_cluster_cloud.reset(new PointCloud());
      for (IndicesIt point = points_vector.begin(); point != points_vector.end(); point++)
      {
         closest_cluster_cloud->push_back((*current_view.cloud)[*point]);
      }
      add_cloud_to_viewer(closest_cluster_cloud, "ClosestCluster", vp_calibration_axes, 255, 0, 0);

      // for now, return the height from the floor plane
      Cluster<Point> clusterPoint(*closest_object_centroid);
      height = std::fabs(clusterPoint.get_distance_to_plane(m_floor_plane)) + DEFAULT_FLOOR_HEIGHT;

      {
         stringstream msg;
         msg << "get_approach(): Calculated a height of " << height;
         m_logger->log(msg);
      }
      
//    return 0;

   }

   int create_surface_map(PointCloud::Ptr surface, const Point & centroid)
   {
      typedef PointCloud::const_iterator Iterator;
      using PCLUtils::dot_double_normalize;
      
  //  std::vector<float> x_values;
  //  std::vector<float> y_values;
      std::vector<std::pair<double, double> > coordinates;
      Point difference;

      float x_diff, y_diff;

      for (Iterator i = surface->begin(); i != surface->end(); ++i)
      {
         // Get the difference
         PCLUtils::subtractXYZ(*i, centroid, difference);

         // Dot the difference with the x vector
         x_diff = dot_double_normalize(difference, m_cal.x_vector);

         // Dot the difference with the y vector
         y_diff = dot_double_normalize(difference, m_cal.y_vector);

         // x/y is backwards in one of the Coordinate.h class or
         // this..
         coordinates.push_back(std::pair<double, double> (y_diff, x_diff));
         
      }

      float slope_a, intercept_b;
      {
         //http://math.stackexchange.com/questions/204020/what-is-the-equation-used-to-calculate-a-linear-trendline
         // CALCULATE SLOPE
         //
         typedef std::vector<std::pair<double, double> >::iterator Iterator;

         float sum_xy   = 0.0;
         float sum_x    = 0.0;
         float sum_y    = 0.0;
         float sum_x2   = 0.0;
         float sum_y2   = 0.0;
         float n        = (float) coordinates.size();

         for (Iterator i = coordinates.begin(); i != coordinates.end();
               i++)
         {
            sum_xy   += i->first * i->second;
            sum_x    += i->first;
            sum_y    += i->second;
            sum_x2   += (i->first * i->first);
            sum_y2   += (i->second * i->second);
         }

         slope_a = (n * sum_xy) - (sum_x * sum_y);
         slope_a = slope_a / (n * sum_x2 - (sum_x*sum_x));
         intercept_b = (sum_y - slope_a * sum_x) / n;
      }

      std::vector<double> trendline(2, 0);
      trendline[1] = slope_a;
      trendline[2] = intercept_b;

      claw_angle = atan(slope_a) * 180 / PI;
      {
         stringstream msg;
         msg << "Calculated slope = " << claw_angle << std::endl;
         m_logger->log(msg);
      }

  //  if (plotter != NULL) {
  //     delete plotter;
  //  }
  //  plotter = new pcl::visualization::PCLPlotter();
  //  plotter->addPlotData(trendline, -4, 4, "trendline");
  //  plotter->addPlotData(coordinates, "coordinates", vtkChart::POINTS);
  //  plotter->plot();
   }

   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
