#include <iostream> // input/output streaming
#include <boost/thread/thread.hpp> // Multithreading
#include <pcl/visualization/pcl_visualizer.h> // Visualizing
#include <pcl/segmentation/sac_segmentation.h> // Segmenting
#include <pcl/io/pcd_io.h> // Read/write from/to file
#include <pcl/filters/extract_indices.h> // Extract indices
#include <pcl/filters/plane_clipper3D.h> // Clip plane
#include <pcl/io/openni_grabber.h> // Get PCs form Kinect
#include <pcl/kdtree/kdtree.h> // KDTree for Euclidean Extraction
#include <pcl/segmentation/extract_clusters.h> // Extract Clusters
#include <pcl/common/centroid.h> // centroid calculation
#include <pcl/filters/project_inliers.h> // projection on to plane

extern "C" { // Readline Stuff
#include <stdio.h> 
#include "readline/readline.h" 
#include "readline/history.h"
}

#include "stringmanip.h" // Custom - Utilities for manipulating strings
#include "mutex.h" // Custom - Lightweight Mutex class
#include "colours.h" // Custom - quick colors struct and definitions

#define VERBOSE

// namespace stuff to reduce typing
using                                                    std::string;
using                                                    std::cout;
using                                                    std::endl;
using                                                    std::cerr;
using                                                    stringmanip::trim;
using                                                    stringmanip::split;
using                                                    stringmanip::arg_list;
typedef  pcl::PointXYZRGBA                               point_t;
using                                                    pcl::CentroidPoint;

// Forward Function Declarations
int                                                      open_file(arg_list args);
int                                                      segment(arg_list args);
int                                                      remove_segment(arg_list args);
int                                                      capture_from_camera(arg_list input);
int                                                      adjust_plane(arg_list input);
int                                                      remove_plane(arg_list input);
int                                                      euclidean_extraction(arg_list input);
void                                                     cluster_lines(pcl::PointCloud<point_t>::Ptr   cloud,
                                                            std::vector<pcl::PointIndices>  &clusters,
                                                            pcl::ModelCoefficients::Ptr     floor_coefficients);

// Program variables and objects
pcl::PointCloud<point_t>::Ptr                            m_cloud;                   
pcl::PointCloud<point_t>::Ptr                            m_cloud_above;                      
pcl::PointCloud<point_t>::Ptr                            m_cloud_below;                      
pcl::PointCloud<point_t>::ConstPtr                       m_cam_cloud;

bool                                                     quit = false;
boost::shared_ptr<pcl::visualization::PCLVisualizer>     viewer;

pcl::ModelCoefficients::Ptr                              floor_coef; 
pcl::ModelCoefficients::Ptr                              rplane_coef; 

bool                                                     received_input = false;
char *                                                   cmdline_input;
string                                                   input;
bool                                                     point_cloud_loaded = false;
bool                                                     got_cam_cloud = false;
bool                                                     got_plane = false;

// Visualization Mutex
Mutex                                                    * upd_vis_mutex;
Mutex                                                    * input_mutex;
Mutex                                                    * cloud_mutex;

pcl::PointIndices::Ptr                                   plane_inliers (new pcl::PointIndices());
pcl::Grabber *                                           interface;

void display_help();



void get_input()
{
   cmdline_input = readline("SHELL > ");
   input = string(cmdline_input);
   input_mutex->lock();
   received_input = true;
   input_mutex->unlock();
}

void _update_cloud_callback(const pcl::PointCloud<point_t>::ConstPtr &cloud)
{
   cloud_mutex->lock();
   m_cam_cloud = cloud; // Is this wrong? A: nope
   got_cam_cloud = true;
   cloud_mutex->unlock();
}

int main(int argc, char ** argv)
{
   // Initialize program Variables and Objects

   // Mutexes
   // Fun fact: Mutex = MUTually EXclusive
   upd_vis_mutex = new Mutex();
   input_mutex   = new Mutex();
   cloud_mutex   = new Mutex();

   // Coefficients
   floor_coef = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());
   rplane_coef = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());

#define NOT_CONNECTING_INTERFACE
#ifndef NOT_CONNECTING_INTERFACE
   // Initialize grabber/Kinect interface. Register callback.
   interface = new pcl::OpenNIGrabber();

   // Note to self - you need to pass in an address to a function for boost::bind
   // boost::bind(__function_name__ ...) will segfault and not tell you why
   // boost::bind(&__function_name__ ...) is the correct call
   boost::function<void (const pcl::PointCloud<point_t>::ConstPtr &)>   callback_function =
      boost::bind(&_update_cloud_callback, _1); 
   
   // Register the callback.
   interface->registerCallback(callback_function);
#endif

   // "Shell is still running"
   bool quit = false;

   // Point Cloud that'll be used to visualize everything
   m_cloud = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>);

   // Viewer
   viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("3D Viewer"));
   viewer->setBackgroundColor(0, 0, 0);

   // ********************************************************************************
   //    Main Loop
   // ********************************************************************************
   while (!quit) {


      // ********************************************************************************
      //       Let user interact with the point cloud visualizer while the program
      //       waits for user input.
      // ********************************************************************************

      boost::thread  input_thread(get_input);

      // IMPORTANT: ANYTHING INVOLVING THE VIEWER NEEDS TO BE IN THE MAIN THREAD.
      // __ANYTHING__ ELSE CAN GO IN A SEPARATE THREAD.
      // DO NOT DO ANYTHING LIKE THIS OUTSIDE OF THE MAIN THREAD!
      while (!viewer->wasStopped())
      {
         viewer->spinOnce(100);

         // Sleep thread for a bit
         boost::this_thread::sleep(boost::posix_time::microseconds(100000));

         input_mutex->lock();
         if (received_input) {
            input_mutex->unlock(); 
            break;
         }
         input_mutex->unlock();
      }

      input_thread.join(); // just to be safe..
      received_input = false;

      // ********************************************************************************
      //       Process Input. Do stuff to point cloud
      // ********************************************************************************
      
      if (trim(input).length() != 0)
      {

         arg_list args = split(input);
         string   cmd_name = args.front();
         args.pop_front();


         if (cmd_name == "load") {
            if (open_file(args) == 0) {
               viewer->addPointCloud<point_t>(m_cloud, "sample cloud");
               viewer->addCoordinateSystem();
            }

         } else if (cmd_name == "quit") {

            quit = true;  // will end the program

         } else if (cmd_name == "segment") {

            segment(args);

         } else if (cmd_name == "remove") {

            remove_segment(args);

         } else if (cmd_name == "eucl") {

            euclidean_extraction(args);

         } else if (cmd_name == "camera") {

            capture_from_camera(args);

         } else if (cmd_name == "plane") {
            
            adjust_plane(args);


         } else {

            if (cmd_name != "help") {
               cout << "Command not recognized." << endl;
            }
            display_help();
         }


      }

      free(cmdline_input); // free the pointer
   }
   return 0;
}

void display_help()
{
   cout << "Commands: " << endl;
   cout << "<load> - load sample cloud" << endl;
   cout << "<quit> - quit program" << endl;
   cout << "<segment> - detect plane" << endl;
   cout << "<remove> - remove plane if detected" << endl;
   cout << "<eucl> - euclidian clustering" << endl;
   cout << "<camera> - retrieve point cloud from Kinect" << endl;
   cout << "<plane> - segment point clud into an 'above/below' plane" << endl;
}

int segment_camera(arg_list input) {
   return 0;
}


int segment(arg_list input)
{
   if (!point_cloud_loaded) {
      cerr << "No point cloud has been loaded! Not segmenting.." << endl;
      return -1;
   }

   // Segmentation object
   pcl::SACSegmentation<point_t>       seg;
   
   // Optional? TODO investigate
   seg.setOptimizeCoefficients(true);

   // Configure segmentation object.
   seg.setModelType(pcl::SACMODEL_PLANE);
   seg.setMethodType (pcl::SAC_RANSAC);
   seg.setMaxIterations (1000);
   seg.setDistanceThreshold (0.01);
 
   // Perform Segmentation
   seg.setInputCloud(m_cloud);
   seg.segment(*plane_inliers, *floor_coef);

   std::cout << "Point Cloud representing plane: " << plane_inliers->indices.size() << " points, out of " << m_cloud->points.size() << " points." << endl;

   if (plane_inliers->indices.size() == 0) {
      cout << "Didn't find a plane!" << endl;
      return -1;
   } else {

      // Colour points in the plane. // TODO make a convenience method
      for (std::vector<int>::const_iterator it = plane_inliers->indices.begin(); it != plane_inliers->indices.end(); ++it)
      {
         m_cloud->points[*it].r = 0;
         m_cloud->points[*it].g = 0;
         m_cloud->points[*it].b = 255;
      }

      // Update the cloud, we've changed colors.
      viewer->updatePointCloud(m_cloud, "sample cloud");

      got_plane = true;
   }

   return 0;
}


/* 
 * ======================================================================================
   Function       : open_file
   Description    : 
      - Loads a custom .pcd file into "m_cloud". This will update the visualizer
        with a rendering of said cloud.
        All points in m_cloud are coloured white (255, 255, 255).
   Arguments      : 
      - arg_list  : a list of arguments to the function, currently ignored.
                    TODO take in a filename.
   Returns        : 0 if successsful, non-zero if there was an error loading the file.
   ====================================================================================== 
*/
int open_file(arg_list input)
{
   const string filename = "../../../data/saved_kinect_pcd.pcd";
   cout << "Opening: " << filename << endl;

   if (pcl::io::loadPCDFile<point_t>(filename, *m_cloud) == -1) {
      cout << "Could not open: <" << filename << ">!" << endl;
      return -1;
   } else {

      // Go through the entire cloud and make evertyhing white..
      cerr << "Warning: colouring everything white.." << endl;
      for (pcl::PointCloud<point_t>::iterator it = m_cloud->begin(); it != m_cloud->end(); ++it)
      {
         (*it).r = 255;
         (*it).g = 255;
         (*it).b = 255;
      }
      
      // If we got the point cloud, set point_cloud_loaded = true.
      point_cloud_loaded = true;

      return 0;
   }
}

/* 
 * ======================================================================================
   Function       : remove_segment
   Description    : Removes all points at the indices specified in "plane_inliers" from 
                    the point cloud "m_cloud".
   Arguments      : 
      - arg_list  : A list of arguments. Not used by this function.
   Returns        : 0 if successful.
   ====================================================================================== 
*/
int remove_segment(arg_list input)
{
   pcl::ExtractIndices<point_t>     extract;

   extract.setIndices(plane_inliers);
   extract.setNegative(true);

   extract.filterDirectly(m_cloud);

   viewer->updatePointCloud(m_cloud, "sample cloud");

   return 0;
}


/* 
 * ======================================================================================
   Function       : capture_from_camera
   Description    : Overwrites "m_cloud" with data from the Kinect.
   Arguments      : arg_list : a list of arguments that is ignored.
   Returns        : 0 if successful.
   ====================================================================================== 
*/
int capture_from_camera(arg_list input) {
   
   cloud_mutex->lock();
   got_cam_cloud = false;
   cloud_mutex->unlock();

   bool last = false;

   interface->start();

   while (!last) 
   {
      cloud_mutex->lock();
      if (got_cam_cloud == true) {
         last = got_cam_cloud;
      }
      cloud_mutex->unlock();
   }

   cout << "Got a point cloud from the camera." << endl;

   *m_cloud = *m_cam_cloud;

   viewer->updatePointCloud(m_cloud, "sample cloud");

   point_cloud_loaded = true;

   interface->stop();

   return 0;
}

/* 
 * ======================================================================================
   Function       : adjust_plane
   Description    : Segments m_cloud into a portion above the plane and below the plane.
   Arguments      : arg_list : ignored.
   Returns        : 0 if successful.
   ====================================================================================== 
*/
int adjust_plane(arg_list input) {

   if (!got_plane || floor_coef->values.size() == 0) {
      printf("Plane not yet found. [DEBUG: floor_coef->values.size() = %0lu]\n", floor_coef->values.size());
      return -1;
   }

   // Remove the plane if it exists..
   viewer->removeShape("rplane");

   cout << "Test: " << input.front() << endl;

   // Copy it over
   rplane_coef->values = floor_coef->values;

   double move_plane_amt = atof(input.front().c_str());
   double new_d_value    = rplane_coef->values[3] + move_plane_amt;

   printf("Original D = <%f>. Moving by <%f> to <%f>.\n", rplane_coef->values[3], move_plane_amt, new_d_value);

   rplane_coef->values[3] = new_d_value;
   viewer->addPlane(*rplane_coef, "rplane");


   // Segment everything above the plane, everything below the plane.

   Eigen::Vector4f   plane_params;
   plane_params << rplane_coef->values[0], rplane_coef->values[1], rplane_coef->values[2], rplane_coef->values[3];

   pcl::PlaneClipper3D<point_t> clipper(plane_params);

   pcl::PointIndices::Ptr below_plane_indices(new pcl::PointIndices());
   pcl::PointIndices::Ptr above_plane_indices(new pcl::PointIndices());

   clipper.clipPointCloud3D(*m_cloud, below_plane_indices->indices);
   printf("After clipping: %0lu points below the plane.\n", below_plane_indices->indices.size());

   viewer->removePointCloud("sample cloud");

   m_cloud_above = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>);
   m_cloud_below = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>);

   // Extract points above the floor into a point cloud.
   pcl::ExtractIndices<point_t>  extract_above;
   extract_above.setIndices(below_plane_indices);
   extract_above.setInputCloud(m_cloud);
   extract_above.setNegative(true);
   extract_above.filter(*m_cloud_above);

   // Colour everything in m_cloud_above GREEN
   for (pcl::PointCloud<point_t>::iterator it = m_cloud_above->begin(); it != m_cloud_above->end(); ++it)
   {
      (*it).r = 0;
      (*it).g = 255;
      (*it).b = 0;
   }


   // Extract points below the floor into a point cloud.
   pcl::ExtractIndices<point_t>  extract_below;
   extract_below.setIndices(below_plane_indices);
   extract_below.setInputCloud(m_cloud);
   extract_below.setNegative(false);
   extract_below.filter(*m_cloud_below);

   // Colour everything in m_cloud_below RED
   for (pcl::PointCloud<point_t>::iterator it = m_cloud_below->begin(); it != m_cloud_below->end(); ++it)
   {
      (*it).r = 255;
      (*it).g = 0;
      (*it).b = 0;
   }

   viewer->addPointCloud<point_t>(m_cloud_above, "abovePlane");
   viewer->addPointCloud<point_t>(m_cloud_below, "belowPlane");
   viewer->updatePointCloud(m_cloud_above, "abovePlane");
   viewer->updatePointCloud(m_cloud_below, "belowPlane");

   // TODO destroy m_cloud
   return -1;

}

/* 
 * ======================================================================================
   Function       : euclidean_extraction
   Description    : Segments m_cloud using euclidean clustering.
   Arguments      : arg_list : ignored.
   Returns        : 0 if successful.
   ====================================================================================== 
*/
int euclidean_extraction(arg_list input) {

   // RIght now: test results using example code given online.

   // Perform a Euclidean Extraction of two clusters with the point clodu m_cloud_above
   pcl::search::KdTree<point_t>::Ptr   search_tree(new pcl::search::KdTree<point_t>);
   std::vector<pcl::PointIndices>      cluster_indices;

   std::vector<int> nan_indices;

   // Not sure why/how these got in...
   pcl::removeNaNFromPointCloud(*m_cloud_above, *m_cloud_above, nan_indices);
   cout << "`NaN`s in m_cloud_above: " << nan_indices.size() << endl;

   search_tree->setInputCloud(m_cloud_above);

   pcl::EuclideanClusterExtraction<point_t>  ec;
   ec.setClusterTolerance(0.20); // KNOB - 2cm: ~9 clusters. 20cm: ~3 clusters.
   ec.setMinClusterSize(100);
   ec.setMaxClusterSize(25000);
   ec.setSearchMethod(search_tree);
   ec.setInputCloud(m_cloud_above);
   ec.extract(cluster_indices);

   printf("After Euclidean extraction: number of clusters = %0lu\n", cluster_indices.size());

   for (int i = 0; i < cluster_indices.size(); i++)
   {
      // Colour these joints
      // These cluster indices are in m_cloud_above.
                                     
      const int r_val = colours::colours[i].red;
      const int g_val = colours::colours[i].green;
      const int b_val = colours::colours[i].blue;

      for (std::vector<int>::const_iterator it = cluster_indices[i].indices.begin(); it != cluster_indices[i].indices.end(); ++it)
      {
         m_cloud_above->points[*it].r = r_val;
         m_cloud_above->points[*it].g = g_val;
         m_cloud_above->points[*it].b = b_val;
      }

   }

   // Update the point cloud.
   viewer->updatePointCloud(m_cloud_above, "abovePlane");

   // Doing this here...
   cluster_lines(m_cloud_above, cluster_indices, floor_coef);
   
   return 0;
}

// TODO:
//    for each cluster compute its 3-d mean.
//    project that point on to the detected plane; draw a line from the mean to its plane projection.
void cluster_lines(pcl::PointCloud<point_t>::Ptr   cloud,
                   std::vector<pcl::PointIndices>  &clusters,
                   pcl::ModelCoefficients::Ptr     floor_coefficients)
{
   // Compute the means of everything in "clusters".
   const int   num_clusters = clusters.size();
   std::vector<CentroidPoint<point_t> > centroids (num_clusters); // may affect performance; it's also averaging RGBA

#ifdef VERBOSE
   cout << "cluster_lines(): number of clusters = " << num_clusters << endl;
#endif 

   pcl::PointCloud<point_t>::Ptr    centroid_cloud = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>);
   pcl::PointCloud<point_t>::Ptr    centroid_proj = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>);

   for (int clust = 0; clust < num_clusters; clust++)
   {
      for (std::vector<int>::const_iterator it = clusters[clust].indices.begin();
            it != clusters[clust].indices.end();
            ++it)
      {
         centroids[clust].add(cloud->points[*it]);
      }

      // Then "get" the centroid..
      point_t  centroid_point;
      centroids[clust].get(centroid_point);
      centroid_point.r = 255;
      centroid_point.g = 255;
      centroid_point.b = 255;

      // add it to the point cloud...
      centroid_cloud->points.push_back(centroid_point);

      // get its projection on to the main plane
   }

   // Add the centroid cloud to the main viewer
   viewer->addPointCloud<point_t>(centroid_cloud, "centroids");
   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "centroids");

   viewer->removePointCloud("abovePlane");
   viewer->removePointCloud("belowPlane");
   viewer->updatePointCloud(centroid_cloud, "centroids");

   pcl::ProjectInliers<point_t> proj;
   proj.setModelType(pcl::SACMODEL_PLANE);
   proj.setInputCloud(centroid_cloud);
   proj.setModelCoefficients(floor_coefficients);

   proj.filter(*centroid_proj);

   viewer->addPointCloud<point_t>(centroid_proj, "centroid projections");
   for (pcl::PointCloud<point_t>::iterator it = centroid_proj->begin();
         it != centroid_proj->end(); ++it) {
      it->r = colours::colours[colours::PURPLE].red;
      it->g = colours::colours[colours::PURPLE].green;
      it->b = colours::colours[colours::PURPLE].blue;
   }
   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "centroid projections");
   viewer->updatePointCloud(centroid_proj, "centroid projections");

   // And draw lines

   // TODO unique identifiers for lines
   for (int clust = 0; clust < num_clusters; clust++)
   {
      viewer->addLine(centroid_cloud->points[clust], centroid_proj->points[clust], 
            colours::colours[clust % NUM_COLOURS].red,
            colours::colours[clust % NUM_COLOURS].green,
            colours::colours[clust % NUM_COLOURS].blue);
   }
}






