
#include "Shell.h"
#include "Mutex.h"
#include "TwoPlaneFilter.h"
#include "Camera.h"
#include "common_defs.h"
#include "Cluster.h"
#include <iostream>

#include "Engine.h"

#include "RobotExt.h"

#define RESOURCES

using std::string;
using pcl::visualization::PointCloudColorHandlerCustom;

class TestVectorShell : public Shell {

public :

   typedef pcl::PointXYZRGBA           Point;
   typedef pcl::PointCloud<Point>      PointCloud;

   Mutex *              mutex;
   PointCloud::Ptr      camcloud;
   Logger *             logger;

   RobotExt             robot;

   Camera<Point>     camera;


   // TODO couple these
   Point          xRegVector, yRegVector;
   float          xRegDelta,  yRegDelta;

   Point planeNormal;

   PointCloud::Ptr      rcloud_src;
   PointCloud::Ptr      rcloud_tgt;

   int vp_left, vp_right;
   int vp_calibration_vectors, vp_navigation_clouds, vp_calibration_clouds, vp_topright;

   PointCloud::Ptr      clusters;
   
   Eigen::Matrix4f      transform_mtx;

   // Just in case someone dynamically declares..
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW


   // Constructor
   TestVectorShell() :
      Shell(),
      mutex(new Mutex()),
      camcloud(new PointCloud()),
      logger(new Logger()),
//    engine(new Engine(camcloud, mutex, logger)),
      robot("/dev/cu.usbserial"),
      camera(camcloud, mutex)
   {
      bind_functions();
      viewer->createViewPort(0.0, 0.0, 0.5, 0.5, vp_calibration_clouds);
      viewer->createViewPort(0.5, 0.0, 1.0, 0.5, vp_topright);
      viewer->createViewPort(0.0, 0.5, 0.5, 1.0, vp_navigation_clouds);
      viewer->createViewPort(0.5, 0.5, 1.0, 1.0, vp_calibration_vectors);


      clusters.reset(new PointCloud());
   }

   void bind_functions()
   {

      Shell::callback pos = boost::bind(&TestVectorShell::pos, this, _1);
      register_function(pos, "pos", "pos");

      Shell::callback home = boost::bind(&TestVectorShell::home, this, _1);
      register_function(home, "home", "home");

      Shell::callback move = boost::bind(&TestVectorShell::move, this, _1);
      register_function(move, "move", "move");

      Shell::callback regX = boost::bind(&TestVectorShell::regX, this, _1);
      register_function(regX, "regX", "regX");

      Shell::callback regY = boost::bind(&TestVectorShell::regY, this, _1);
      register_function(regY, "regY", "regY");

      Shell::callback analyze = boost::bind(&TestVectorShell::analyze, this, _1);
      register_function(analyze, "ana", "ana");

      Shell::callback mcc = boost::bind(&TestVectorShell::move_to_closest_cluster, this, _1);
      register_function(mcc, "mcc", "mcc");
   }

   void pos(arg_list args)
   {
      // get current position.
      const    RobotPosition current = robot.currentPos();
      const    RobotPosition max     = robot.currentLimits().max();
      const    RobotPosition min     = robot.currentLimits().min();
      
      // cout << sprintf("") << endl;


      printf("%-10s | %-10s | %-10s | %-10s\n", "AXIS", "MIN", "CURRENT", "MAX");
      printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "X", min.x, current.x, max.x);
      printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "Y", min.y, current.y, max.y);
      printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "Z", min.z, current.z, max.z);
      printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "j4", min.j4, current.j4, max.j4);
      printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "j5", min.j5, current.j5, max.j5);
      printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "j6", min.j6, current.j6, max.j6);
      
   }



   void home(arg_list args)
   {
      robot.home(false);
   }

   void move(arg_list args)
   {
      RobotPosition new_pos = robot.currentPos();

      while (!args.empty()) {
         string axis = args.front();
         float value;
         args.pop_front();
         if (args.empty()) {
            cout << "Illegal command." << endl;
            return;
         } else {
            value = atof(args.front().c_str());
            args.pop_front();
         }

         if      (axis == "x")      new_pos.x = value;
         else if (axis == "y")      new_pos.y = value;
         else if (axis == "z")      new_pos.z = value;
         else {
            cout << "Unrecognized argument: <" << axis << ">" << endl;
            return;
         }
      }

      // Output the position to the console
      cout << "NEW POSITION:" << endl;
      cout << new_pos << endl;

      robot.moveTo(new_pos);

      // Poll the buffer
      string reply;
      robot.controller >> reply;

      cout << "REPLY:" << endl << reply << endl;

   }

   void moveXRel(float amt)
   {
      RobotPosition new_pos = robot.currentPos();
      new_pos.x = new_pos.x + amt;

      // TODO check validity of new position
      std::cerr << "TODO: moveXRel() - Implement a validity check of the new X Location!" << std::endl;

      xRegDelta = amt;

      cout << "New Position: " << endl;
      cout << new_pos << endl;

      robot.moveTo(new_pos);

      string reply;
      robot.controller >> reply;
      if (reply != "") {
         cout << "Reply: " << endl << reply << endl;
      }
   }

   void moveYRel(float amt)
   {
   {
      RobotPosition new_pos = robot.currentPos();
      new_pos.y = new_pos.y + amt;

      // TODO check validity of new position
      std::cerr << "TODO: moveYRel() - Implement a validity check of the new Y Location!" << std::endl;

      yRegDelta = amt;

      cout << "New Position: " << endl;
      cout << new_pos << endl;

      robot.moveTo(new_pos);

      string reply;
      robot.controller >> reply;
      if (reply != "") {
         cout << "Reply: " << endl << reply << endl;
      }
   }
   }



   // TODO move to utils
   void subtractXYZ(const Point & _this_, const Point & _minus_this_, Point & _equals_this_)
   {
      _equals_this_.x = _this_.x - _minus_this_.x;
      _equals_this_.y = _this_.y - _minus_this_.y;
      _equals_this_.z = _this_.z - _minus_this_.z;
   }
   // TODO move to utils
   void addXYZ(const Point & _this_, const Point & _plus_this_, Point & _equals_this_)
   {
      _equals_this_.x = _this_.x + _plus_this_.x;
      _equals_this_.y = _this_.y + _plus_this_.y;
      _equals_this_.z = _this_.z + _plus_this_.z;
   }

   void load1(arg_list args)
   {
      viewer->removePointCloud("RCloudOneUnfiltered");
      rcloud_src.reset(new PointCloud);
      camera.retrieve();
      *rcloud_src = *camcloud;
         PointCloudColorHandlerCustom<Point> handler_red_1(rcloud_src, 255, 0, 0);
         viewer->addPointCloud(rcloud_src, handler_red_1, "RCloudOneUnfiltered", vp_calibration_clouds);
   }


   void load2(arg_list args)
   {
      viewer->removePointCloud("RCloudTwoUnfiltered");
      rcloud_tgt.reset(new PointCloud);
      camera.retrieve();
      *rcloud_tgt = *camcloud;
         PointCloudColorHandlerCustom<Point> handler_blue_1(rcloud_tgt, 0, 0, 255);
         viewer->addPointCloud(rcloud_tgt, handler_blue_1, "RCloudTwoUnfiltered", vp_calibration_clouds);
   }

   

   void scene_setup(int viewport, Eigen::Vector4f & plane_eigen, PointCloud::Ptr clusters)
   {
      Point origin; origin.x = 0.0, origin.y = 0.0, origin.z = 0.0;
      Cluster<Point> originCluster(origin);

      Point originProjection = originCluster.get_plane_projection(plane_eigen);

      viewer->removeShape("originLine", viewport);
      viewer->addLine(origin, originProjection, "originLine", viewport);

      viewer->removePointCloud("clusterLocations", viewport);
      viewer->addPointCloud(clusters, "Clusters", viewport);
   }

      // NO REGISTRATION - only using clusters
   bool clusterDifference(Point & difference,
      PointCloud::Ptr source_cloud,
      PointCloud::Ptr target_cloud,
      Eigen::Vector4f & floor_plane,
      PointCloud::Ptr clusterCloud)
   {
      Mutex m;

      // Remove Plane (floor) from point clouds.
      {
         TwoPlaneFilter<Point> tpOne(source_cloud, &m);
         tpOne.filter_plane();
         TwoPlaneFilter<Point> tpTwo(target_cloud, &m);
         tpTwo.filter_plane();

         floor_plane = tpTwo.get_plane_coefficients();
      }

      // Locate clusterCloud.
      ClusterFinder<Point> cfOne(source_cloud, &m);
      cfOne.find_clusters();
      PointCloud::Ptr      source_clusters = cfOne.get_clusters();
      std::cout << "rlcoud_src: Found " << source_clusters->size() << " clusterCloud." << std::endl;

      ClusterFinder<Point> cfTwo(target_cloud, &m);
      cfTwo.find_clusters();
      PointCloud::Ptr      target_clusters = cfTwo.get_clusters();
      std::cout << "target_cloud: Found " << target_clusters->size() << " clusterCloud." << std::endl;

      if (source_clusters->size() != 1 || target_clusters->size() != 1)
      {
         std::cerr << "clusterDifference() - located more than one cluster in one of the point clouds." << std::endl;
         std::cerr << "Returning false." << std::endl;
         return FAILURE;
      }

      Point source_cluster = source_clusters->points[0];
      Point target_cluster = target_clusters->points[0];

      source_cluster.r = 255; source_cluster.g = 0; source_cluster.b = 0;
      target_cluster.r = 0; target_cluster.g = 0; target_cluster.b = 255;

      clusterCloud->push_back(source_cluster);
      clusterCloud->push_back(target_cluster);

      subtractXYZ(target_cluster, source_cluster, difference);

      return SUCCESS;
   }

   void regX(arg_list args)
   {
      load1(args); // Load first point cloud and display.
   // move(args);

      float move_x_amt;
      // Pop arguments off
      if (args.empty()) { 
         cout << "No arguments provided. Returning." << endl;
         return;
      } else {
         move_x_amt = atof(args.front().c_str());
         if (move_x_amt != move_x_amt) {
            cout << "Illegal argument. Exiting." << endl;
            return;
         }
      }

      moveXRel(move_x_amt);
      
      load2(args); // Load second point cloud and display.

//    Point difference;
      Eigen::Vector4f floor_plane;

      PointCloud::Ptr clusters(new PointCloud());

      bool result = clusterDifference(xRegVector, rcloud_src, rcloud_tgt, floor_plane, clusters);

      planeNormal.x = floor_plane(0);
      planeNormal.y = floor_plane(1);
      planeNormal.z = floor_plane(2);

      // Visualize the clusters. Source = red, Target = blue.

      Point difference_fp; // Differnce, on floor plane

      Cluster<Point> originCluster(0.0, 0.0, 0.0);
      Point originProjection = originCluster.get_plane_projection(floor_plane);
      addXYZ(originProjection, xRegVector, difference_fp);
      viewer->removeShape("XRegLine");
      viewer->addLine(originProjection, difference_fp, 214, 0, 255, "XRegLine", vp_calibration_vectors); // magenta

      scene_setup(vp_calibration_vectors, floor_plane, clusters);
   }

   void regY(arg_list args)
   {
      load1(args);
//    move(args);

      float move_y_amt;
      // Pop arguments off
      if (args.empty()) { 
         cout << "No arguments provided. Returning." << endl;
         return;
      } else {
         move_y_amt = atof(args.front().c_str());
         if (move_y_amt != move_y_amt) {
            cout << "Illegal argument. Exiting." << endl;
            return;
         }
      }
      moveYRel(move_y_amt);

      load2(args);

      Point difference;
      Eigen::Vector4f floor_plane;
      
      PointCloud::Ptr clusters(new PointCloud());
      bool result = clusterDifference(yRegVector, rcloud_src, rcloud_tgt, floor_plane, clusters);

      planeNormal.x = floor_plane(0);
      planeNormal.y = floor_plane(1);
      planeNormal.z = floor_plane(2);

      Point difference_fp; // Difference, on floor plane
      Cluster<Point> originCluster(0.0, 0.0, 0.0);
      Point originProjection = originCluster.get_plane_projection(floor_plane);

      addXYZ(originProjection, yRegVector, difference_fp);
      viewer->removeShape("YRegLine");
      viewer->addLine(originProjection, difference_fp, 255, 165, 0, "YRegLine", vp_calibration_vectors); // yellow

      viewer->updatePointCloud(clusters, "Clusters");
   }

   void analyze(arg_list args)
   {
      float xy_angle_rads = PCLUtils::angleBetween(xRegVector, yRegVector);
      float xy_angle_degrees = acos(xy_angle_rads) * 180.0 / 3.14159;

      std::cout << "xy_angle_degrees = " << xy_angle_degrees << std::endl;

      float xz_angle_rads = PCLUtils::angleBetween(xRegVector, planeNormal);
      float xz_angle_degrees = acos(xz_angle_rads) * 180.0 / 3.14159;

      std::cout << "xz_angle_degrees = " << xz_angle_degrees << std::endl;

      float yz_angle_rads = PCLUtils::angleBetween(yRegVector, planeNormal);
      float yz_angle_degrees = acos(yz_angle_rads) * 180.0 / 3.14159;

      std::cout << "yz_angle_degrees = " << yz_angle_degrees << std::endl;
   }


   void move_to_closest_cluster(arg_list args)
   {
      /*
       * Retrieve Point Cloud

       * Filter plane and find clusters
       * iterate through clusters and find the closest on to the "clawPoint"
       *    (currently (0, 0, 0) but this shoudl change...
       * Get the vector from the "clawPoint" to the closest cluster.
       * project it on to the X and Y basis vectors.
       * Translate the projections into a RobotPosition using the positions
       * from calibration.
       * Move to that position.
       */

      // Show: 
      //    Filtered Cloud
      //    Centroids
      //    Projections
      //    Line from Origin to plane

      // Use the "botright" viewport.

      // Retrieve Point Cloud
      PointCloud::Ptr      origCloud(new PointCloud);
      PointCloud::Ptr      clusterCloud;
      camera.retrieve();
      *origCloud = *camcloud;


      // Filter plane and find clusters
      Mutex m; 
      
      TwoPlaneFilter<Point> filt(origCloud, &m);
      filt.filter_plane();

      PointCloudColorHandlerCustom<Point>    handler_cyan(origCloud, 0, 255, 255);
      viewer->removePointCloud("TestCloud");
      viewer->addPointCloud(origCloud, handler_cyan, "TestCloud", vp_navigation_clouds);



      ClusterFinder<Point> finder(origCloud, &m);
      finder.find_clusters();
      clusterCloud = finder.get_clusters();

      // Add 
      //

      if (clusterCloud->size() != 1 ) {
         std::cerr << "move_to_closest_cluster() only implemented for a single cluster." << endl;
         return;
      }

      // Project the cluster on to the basis vectors
      Point    closestCluster = clusterCloud->points[0];
      Cluster<Point> originCluster(0.0, 0.0, 0.0);
      Point    originProjection = originCluster.get_plane_projection(filt.get_plane_coefficients());

      Point closestClusterVec;

      subtractXYZ(closestCluster, originProjection, closestClusterVec);

      Point origin; origin.x = 0.0, origin.y = 0.0, origin.z = 0.0;
//    Point    yRegPlane, xRegPlane;
//    addXYZ(yRegVector, originProjection, yRegPlane);
//    addXYZ(xRegVector, originProjection, xRegPlane);

      float    x_amt = PCLUtils::dot_double_normalize(closestClusterVec, xRegVector);
      float    y_amt = PCLUtils::dot_double_normalize(closestClusterVec, yRegVector);

      viewer->removeShape("NavOriginLine");
      viewer->addLine(origin, originProjection, "NavOriginLine", vp_navigation_clouds);
      viewer->addLine(originProjection, closestCluster, 232, 232, 16, "NavOriginLine", vp_navigation_clouds);


      viewer->removeShape("ClosestClusterLine");
      viewer->addLine(originProjection, closestCluster, 232, 16, 221, "ClosestClusterLine", vp_navigation_clouds);

      cout << "Normalized X: " << x_amt << endl;
      cout << "Normalized Y: " << y_amt << endl;

      RobotPosition newPosGuess = robot.currentPos();
      newPosGuess.x = newPosGuess.x + (-1.0 * x_amt * xRegDelta);
      newPosGuess.y = newPosGuess.y + (-1.0 * y_amt * yRegDelta);


      cout << "Moving to guess. Remember to check validity..." << endl;
      robot.moveTo(newPosGuess);

      cout << "New Position Guess: " << endl;
      cout << newPosGuess << endl;
   }

};


int main (int argc, char ** argv)
{
   TestVectorShell   ts;
   ts.run_shell();
   return SUCCESS;
};
