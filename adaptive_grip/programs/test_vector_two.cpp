#include "Shell.h"
#include "Mutex.h"
#include "TwoPlaneFilter.h"
#include "Camera.h"
#include "common_defs.h"
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

#ifdef RESOURCES
   RobotExt             robot;
// Engine           *   engine;
#endif

   Camera<Point>     camera;



   PointCloud::Ptr      rcloud_src;
   PointCloud::Ptr      rcloud_tgt;

   int vp_left, vp_right;
   int vp_botright, vp_botleft, vp_topleft, vp_topright;

   
   Eigen::Matrix4f      transform_mtx;

   // Just in case someone dynamically declares..
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW


   // Constructor
   TestVectorShell() :
      Shell(),
      mutex(new Mutex()),
      camcloud(new PointCloud()),
      logger(new Logger()),
#ifdef RESOURCES
//    engine(new Engine(camcloud, mutex, logger)),
      robot("/dev/cu.usbserial"),
#endif 
      camera(camcloud, mutex)
   {
      bind_functions();
  //  viewer->createViewPort(0.0, 0.0, 1.0, 0.5, vp_left);
  //  viewer->createViewPort(0.0, 0.5, 1.0, 1.0, vp_right);
      viewer->createViewPort(0.0, 0.0, 0.5, 0.5, vp_topleft);
      viewer->createViewPort(0.5, 0.0, 1.0, 0.5, vp_topright);
      viewer->createViewPort(0.0, 0.5, 0.5, 1.0, vp_botleft);
      viewer->createViewPort(0.5, 0.5, 1.0, 1.0, vp_botright);

   }

   void bind_functions()
   {
#ifdef RESOURCES
//    Shell::callback get_clusters = boost::bind(&TestVectorShell::get_clusters, this, _1);
//    register_function(get_clusters, "get_clusters", "Get Clusters");

      Shell::callback get_image = boost::bind(&TestVectorShell::get_image, this, _1);
      register_function(get_image, "image", "Get Image");

      Shell::callback pos = boost::bind(&TestVectorShell::pos, this, _1);
      register_function(pos, "pos", "pos");

      Shell::callback home = boost::bind(&TestVectorShell::home, this, _1);
      register_function(home, "home", "home");

      Shell::callback move = boost::bind(&TestVectorShell::move, this, _1);
      register_function(move, "move", "move");


      Shell::callback reg3 = boost::bind(&TestVectorShell::reg3, this, _1);
      register_function(reg3, "reg3", "reg3");

      Shell::callback load1 = boost::bind(&TestVectorShell::load1, this, _1);
      register_function(load1, "load1", "load1");

      Shell::callback load2 = boost::bind(&TestVectorShell::load2, this, _1);
      register_function(load2, "load2", "load2");
#endif
      Shell::callback reg2 = boost::bind(&TestVectorShell::reg2, this, _1);
      register_function(reg2, "reg2", "reg2");

   }


#ifdef RESOURCES 
   void get_image(arg_list args)
   {
  //  engine->get_image();
  //  viewer->addPointCloud(engine->m_vis_cloud, "VisualizationCloud", vp_left);
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

   void claw(arg_list args)
   {
      
   }

   void load1(arg_list args)
   {
      viewer->removePointCloud("RCloudOneUnfiltered");
      rcloud_src.reset(new PointCloud);
      camera.retrieve();
      *rcloud_src = *camcloud;
         PointCloudColorHandlerCustom<Point> handler_red_1(rcloud_src, 255, 0, 0);
         viewer->addPointCloud(rcloud_src, handler_red_1, "RCloudOneUnfiltered", vp_topleft);
   }


   void load2(arg_list args)
   {
      viewer->removePointCloud("RCloudTwoUnfiltered");
      rcloud_tgt.reset(new PointCloud);
      camera.retrieve();
      *rcloud_tgt = *camcloud;
         PointCloudColorHandlerCustom<Point> handler_blue_1(rcloud_tgt, 0, 0, 255);
         viewer->addPointCloud(rcloud_tgt, handler_blue_1, "RCloudTwoUnfiltered", vp_topleft);
   }

   void reg3(arg_list args)
   {
      PointCloud::Ptr      reg_output(new PointCloud);
      Mutex                m;
      Eigen::Matrix4f      transform;
      /* Capture rcloud_src and rcloud_tgt */
//    viewer->removePointCloud("RCloudOne");
//    viewer->removePointCloud("RCloudTwo");

//    rcloud_src.reset(new PointCloud);
//    Camera<Point> cam (cam_buffer, &m);
// // engine->get_image();
//    cam.retrieve();
//    *rcloud_src = *cam_buffer;

//    move(args);

//    rcloud_tgt.reset(new PointCloud);
//    engine->get_image();
//    cam.retrieve();
//    *rcloud_tgt = *cam_buffer;

      
      
      /* Display the unfilitered point clouds on the top right.. */
      std::cout <<  "Before filter: rcloud_src->size() = " << rcloud_src->size() << std::endl;
      std::cout <<  "Before filter: rcloud_tgt->size() = " << rcloud_tgt->size() << std::endl;

      /* Filter the Point Clouds */
      TwoPlaneFilter<Point> tp2(rcloud_tgt, &m);
      tp2.filter_plane();
      TwoPlaneFilter<Point> tp1(rcloud_src, &m);
//    tp2.set_input_cloud(rcloud_src);
      tp1.filter_plane();
      // TODO there's an error with using the same TwoPlaneFilter with two different point clouds..


//    return;

      /* Display original point clouds on bottom right. */
     
      PointCloudColorHandlerCustom<Point> handler_red_2(rcloud_src, 255, 0, 0);
      viewer->addPointCloud(rcloud_src, handler_red_2, "RCloudOne", vp_botright);
      PointCloudColorHandlerCustom<Point> handler_blue_2(rcloud_tgt, 0, 0, 255);
      viewer->addPointCloud(rcloud_tgt, handler_blue_2, "RCloudTwo", vp_botright);
      

      std::cout <<  "After filter: rcloud_src->size() = " << rcloud_src->size() << std::endl;
      std::cout <<  "After filter: rcloud_tgt->size() = " << rcloud_tgt->size() << std::endl;


      /* Then, try to register the point clouds */
      PCLUtils::pairAlign<Point>(rcloud_src, rcloud_tgt, reg_output, transform, logger);

      // Scratch:
      // by applying the "transform" to cf2 (target), you get back the points in cf1 (source).
      // the "transform" is the target-to-source transform. Can invert it (Matrix.inverse()) to get
      // source-to-target.
      //
      /* Display the source centroids (blue),
       * Target cloud centroids (red),
       * and estimate-target centroids (Green),
       *
       * LIMIT to one centroid */
      ClusterFinder<Point>    cf(rcloud_src, &m);
      cf.find_clusters();
      PointCloud::Ptr clusters_src = cf.get_clusters();
      std::cout << "clusters_src.size() = " << clusters_src->size() << std::endl;

      ClusterFinder<Point>    cf2(rcloud_tgt, &m);
    //cf.setInputCloud(rcloud_tgt);
      cf2.find_clusters();
      PointCloud::Ptr clusters_tgt = cf2.get_clusters();
      std::cout << "clusters_tgt.size() = " << clusters_tgt->size() << std::endl;
      // TODO same thing with the clusterFinder. use two of them.

      /* Calculate the estimate of the source */
      Eigen::Affine3f transform_affine;
      transform_affine.matrix() = transform.matrix();
      PointCloud::Ptr   src_centroids_est(new PointCloud);
      pcl::transformPointCloud(*clusters_src, *src_centroids_est, transform_affine);

      Eigen::Vector4f tmp_plane = tp2.get_plane_coefficients();
      int lineNum = 0;
      std::cout << "From clusters_tgt..." << std::endl;
      for (std::vector<Point>::iterator i = clusters_tgt->begin();
            i != clusters_tgt->end(); i++)
      {
         Cluster<Point> c (*i);
         Point    projection = c.get_plane_projection(tmp_plane);
         std::stringstream lineName;
         lineName << "Line" << (lineNum++);
         viewer->addLine(c.m_location, projection, lineName.str().c_str(), vp_botleft);
         // ALSO, OUTPUT THE POINT...
         std::cout << "CLUSTER " << lineNum << ": x = " << (*i).x <<
            ", y = " << (*i).y << ", z = " << (*i).z << std::endl;
      }

      std::cout << "From clusters_src..." << std::endl;
      for (std::vector<Point>::iterator i = clusters_src->begin();
            i != clusters_src->end(); i++)
      {
         Cluster<Point> c (*i);
         Point    projection = c.get_plane_projection(tmp_plane);
         std::stringstream lineName;
         lineName << "Line" << (lineNum++);
         viewer->addLine(c.m_location, projection, lineName.str().c_str(), vp_botleft);
         std::cout << "CLUSTER " << lineNum << ": x = " << (*i).x <<
            ", y = " << (*i).y << ", z = " << (*i).z << std::endl;
      }

//    viewer->addPlane(*(tp2.m_plane), "tp2.m_plane");

      Point origin; origin.x = 0.0, origin.y = 0.0, origin.z = 0.0;
      origin.r = 255; origin.g = 255; origin.b = 255;
      Cluster<Point> originCluster(origin);
      Point originProjection = originCluster.get_plane_projection(tmp_plane);
      viewer->addLine(origin, originProjection, "originLine", vp_botleft);
      

      /* Now Show the Estimate Centroids (Green), Source Centroids (Blue), Target Centroids (Red) */
      {
         PointCloudColorHandlerCustom<Point> handler_red(clusters_src, 255, 0, 0);
         viewer->addPointCloud(clusters_src, handler_red, "ClustersSrc", vp_botleft);

         PointCloudColorHandlerCustom<Point> handler_blue(clusters_tgt, 0, 0, 255);
         viewer->addPointCloud(clusters_tgt, handler_blue, "ClustersTgt", vp_botleft);

         

         PointCloudColorHandlerCustom<Point> handler_green(src_centroids_est, 0, 255, 0);
         viewer->addPointCloud(src_centroids_est, handler_green, "ClustersSrcEst", vp_botleft);

         viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ClustersSrc");
         viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ClustersTgt");
         viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ClustersSrcEst");
      }
      

      if (clusters_src->size() != 1 || clusters_tgt->size() != 1)
      {
         std::cerr << "Error - more than one cluster in source/target cluster clouds. " << std::endl;
         std::cerr << "When attempting to get vector from source clusters to target cluster" << std::endl;
         return;
      }

      pcl::PointXYZ tmp_originProjection; tmp_originProjection.x = originProjection.x;
      tmp_originProjection.y = originProjection.y;
      tmp_originProjection.z = originProjection.z;


      pcl::PointXYZ source_to_target;
      source_to_target.x = clusters_tgt->points[0].x - clusters_src->points[0].x;
      source_to_target.y = clusters_tgt->points[0].y - clusters_src->points[0].y;
      source_to_target.z = clusters_tgt->points[0].z - clusters_src->points[0].z;
      
      // The same vector, but starting from the projection of the origin.
      pcl::PointXYZ source_to_target_op;
      source_to_target_op.x = tmp_originProjection.x + source_to_target.x;
      source_to_target_op.y = tmp_originProjection.y + source_to_target.y;
      source_to_target_op.z = tmp_originProjection.z + source_to_target.z;
      viewer->addLine(tmp_originProjection, source_to_target_op, 255, 0, 0, "vec", vp_botleft);


   }
#endif // RESOURCES

   void reg2(arg_list args)
   {

      // ============================================================================================================
      // Initialize rcloud_src and r
      // ============================================================================================================
      rcloud_src.reset(new PointCloud);
      rcloud_tgt.reset(new PointCloud);
      // Get rcloud_src and rcloud_tgt from CameraFile objects
      PointCloud::Ptr   tmpCloud(new PointCloud);
      Mutex             tmp;
      CameraFile<Point> cam(tmpCloud, &tmp);

      // ============================================================================================================
      // Retrieve rcloud_src and rcloud_tgt
      // ============================================================================================================
      cam.setFilename("/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/data/registration/ball_one.pcd");
      cam.retrieve();
      *rcloud_src = *tmpCloud;

      PointCloudColorHandlerCustom<Point> handler_red(rcloud_src, 255, 0, 0);
      viewer->addPointCloud(rcloud_src, handler_red, "RCloudOne", vp_right);

      cam.setFilename("/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/data/registration/ball_two.pcd");
      cam.retrieve();
      *rcloud_tgt = *tmpCloud;

      PointCloudColorHandlerCustom<Point> handler_blue(rcloud_tgt, 0, 0, 255);
      viewer->addPointCloud(rcloud_tgt, handler_blue, "RCloudTwo", vp_right);

      // ============================================================================================================
      // Filter rcloud_src and rcloud_tgt
      // ============================================================================================================
      TwoPlaneFilter<Point> tp1(rcloud_src, &tmp);
      TwoPlaneFilter<Point> tp2(rcloud_tgt, &tmp);
      tp1.filter_plane();
      tp2.filter_plane();

      PointCloud::Ptr  output(new PointCloud);
      Eigen::Matrix4f  transform;

      // ============================================================================================================
      // call pairAlign to get the transform matrix
      // ============================================================================================================
      PCLUtils::pairAlign<Point>(rcloud_src, rcloud_tgt, output, transform, logger);

      // Display line from (0.0) to the plane on vp_left
      Point origin;
      origin.x = 0; origin.y = 0; origin.z = 0;
      Cluster<Point>  originCluster(origin);
      Point originProj = originCluster.get_plane_projection(tp1.get_plane_coefficients());
      viewer->addLine(origin, originProj, "OriginProjection");
      //
      // Display old cluster locations in green
      PointCloud::Ptr   ccloud1(new PointCloud);
      ClusterFinder<Point> cf1(rcloud_src, &tmp);
      int numClust1 = cf1.find_clusters();
      for (std::vector<Point>::const_iterator it = cf1.m_centroids.begin();
            it != cf1.m_centroids.end(); ++it) { std::cout << " x = " << (*it).x << std::endl;
         std::cout << " y = " << (*it).y << std::endl;
         std::cout << " z = " << (*it).z << std::endl;
         ccloud1->push_back(*(it));
      }
      std::cout << "Got " << numClust1 << " centroids from cf1." << std::endl;

      PointCloudColorHandlerCustom<Point> pc_ccloud1(ccloud1, 255, 0, 0);
      viewer->addPointCloud(ccloud1, pc_ccloud1, "Centroids1", vp_left);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Centroids1");


      //
      // Display new cluster locations in red
      PointCloud::Ptr   ccloud2(new PointCloud);
      ClusterFinder<Point> cf2(rcloud_tgt, &tmp);
      int numClust2 = cf2.find_clusters();
      for (std::vector<Point>::const_iterator it = cf2.m_centroids.begin();
            it != cf2.m_centroids.end(); ++it) {
         std::cout << " x = " << (*it).x << std::endl;
         std::cout << " y = " << (*it).y << std::endl;
         std::cout << " z = " << (*it).z << std::endl;
         ccloud2->push_back(*(it));
      }
      std::cout << "Got " << numClust2 << " centroids from cf2." << std::endl;

      PointCloudColorHandlerCustom<Point> pc_ccloud2(ccloud2, 0, 0, 255);
      viewer->addPointCloud(ccloud2, pc_ccloud2, "Centroids2", vp_left);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Centroids2");
      //
      // Display transform vector from intersection of (0.0) line and plane
      // to some arbitrary length outwards
      // TODO this is shit code, do it with  all the centroids
      // Difference is just an estimate since it uses centroids
      pcl::PointXYZ    Difference;
      Difference.x = cf1.m_centroids[0].x - cf2.m_centroids[0].x;
      Difference.y = cf1.m_centroids[0].y - cf2.m_centroids[0].y;
      Difference.z = cf1.m_centroids[0].z - cf2.m_centroids[0].z;
//    std::cout << " x = " << Difference.x << std::endl;
//    std::cout << " y = " << Difference.y << std::endl;
//    std::cout << " z = " << Difference.z << std::endl;

      pcl::PointXYZ centroid1_Difference; 
      centroid1_Difference.x = cf1.m_centroids[0].x + Difference.x;
      centroid1_Difference.y = cf1.m_centroids[0].y + Difference.y;
      centroid1_Difference.z = cf1.m_centroids[0].z + Difference.z;
      // And plot it
//    viewer->addLine(cf1.m_centroids[0], centroid1_Difference, "DifferenceLine", vp_left);
//    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
//          4, "DifferenceLine");

//    std::cout << "Sanity: transform (1,3) = " << transform(1,3) << std::endl;

      // Difference length:
//    float df_length = Difference.x * Difference.x + Difference.y * Difference.y
//       + Difference.z * Difference.z;
//    std::cout << "Difference length: " << df_length << std::endl;

//    pcl::PointXYZ centroid1_Translation;
//    centroid1_Translation.x = cf1.m_centroids[0].x + transform(0,3);
//    centroid1_Translation.y = cf1.m_centroids[0].y + transform(1,3);
//    centroid1_Translation.z = cf1.m_centroids[0].z + transform(2,3);
//    viewer->addLine(cf1.m_centroids[0], centroid1_Translation, "TranslationLine", vp_left);
//    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
//          4, "TranslationLine");

//    // Translation Length:
//    float tl_length = transform(0,3)*transform(0,3)
//       + transform(1,3) * transform(1,3) + transform(2,3) * transform(2, 3);
//    std::cout << "Translation Length: " << tl_length << std::endl;

      // REALLY SHITTY MATRIX MULTIPLICATION : BEGIN
      Eigen::Affine3f tmp_transform;
      tmp_transform.matrix() = transform.matrix();
      Eigen::Vector3f translation_vec = tmp_transform.translation();

      pcl::PointXYZRGBA tmp_xyz;
      tmp_xyz.x = cf2.m_centroids[0].x; tmp_xyz.y = cf2.m_centroids[0].y; tmp_xyz.z = cf2.m_centroids[0].z; 

      pcl::PointXYZRGBA centroid1_transform = pcl::transformPoint(tmp_xyz, tmp_transform);
   // viewer->addLine(cf2.m_centroids[0], centroid1_transform, 255, 0, 0, "TransformationLine", vp_left);
   // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
   //       2, "TransformationLine");

   // viewer->addLine(cf2.m_centroids[0], cf1.m_centroids[0], 0, 0, 255, "DirectLine", vp_left);
   // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
   //       2, "directline");
   //
   //
      PointCloud::Ptr   cloud_estimate(new PointCloud);
      cloud_estimate->push_back(centroid1_transform); //the point
      PointCloudColorHandlerCustom<Point> pc_cloud3(cloud_estimate, 0, 255, 0);
      viewer->addPointCloud(cloud_estimate, pc_cloud3, "Estimate", vp_left);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Estimate");



//    Eigen::Matrix4f translation = tmp_transform.translation();

//    pcl::PointXYZ samplePoint; samplePoint.x = 1; samplePoint.y = 1; samplePoint.z = 1;
//    pcl::PointXYZ result = pcl::transformPoint(samplePoint, tmp_transform);

//    std::cout << "Transforming (1, 1, 1)..." << std::endl;
      std::cout << "Original (cf2.m_centroids[0]:" << std::endl;
      std::cout << " x = " << cf2.m_centroids[0].x << std::endl;
      std::cout << " y = " << cf2.m_centroids[0].y << std::endl;
      std::cout << " z = " << cf2.m_centroids[0].z << std::endl;
      std::cout << "By transforming the centroid.." << std::endl;
      std::cout << " x = " << centroid1_transform.x << std::endl;
      std::cout << " y = " << centroid1_transform.y << std::endl;
      std::cout << " z = " << centroid1_transform.z << std::endl;
      std::cout << "Original (cf1.m_centroids[0]:" << std::endl;
      std::cout << " x = " << cf1.m_centroids[0].x << std::endl;
      std::cout << " y = " << cf1.m_centroids[0].y << std::endl;
      std::cout << " z = " << cf1.m_centroids[0].z << std::endl;
      

   }

};


int main (int argc, char ** argv)
{
   TestVectorShell   ts;
   ts.run_shell();
   return SUCCESS;
};
