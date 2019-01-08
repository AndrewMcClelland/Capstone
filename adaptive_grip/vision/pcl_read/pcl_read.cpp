#include <iostream>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>

using pcl::io::loadPCDFile;
using std::string;
using pcl::PointIndices;


typedef pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_ptr;

// Dummy prompt function to wait for keypress
void prompt(const string str)
{
   do
   {
      cout << str << endl;
      cout << endl << "Press Any Key to continue...";
   } while (std::cin.get() != '\n');
}

// Construct a simple plane to test plane segmentation code.
// This plane can be detected.
void construct_simple_plane(cloud_ptr cloud)
{
   cloud->width   = 15;
   cloud->height  = 15;
   cloud->points.resize(cloud->width * cloud->height);

   for (size_t i = 0; i < cloud->points.size(); i++)
   {
      cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
      cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
      cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
      // TODO find out why cloud->points[i].z = 1; makes a straight line

   }
 
}

// Assumes that the SACSegmentation object being passed in has been constructed and has an input cloud.
size_t scan_for_plane(pcl::SACSegmentation<pcl::PointXYZ> & segmentation,
                   PointIndices::Ptr inliers, 
                   double threshold,
                   pcl::ModelCoefficients::Ptr coefficients) {

   // Set the threshold value..
   segmentation.setDistanceThreshold(threshold);
   
   // Perform segmentation
   segmentation.segment(*inliers, *coefficients);

   // Return the number of inliers; -1 if there was no planar model found.
   if ( inliers->indices.size() == 0) {
      PCL_ERROR("Could not estimate a planar model for the given dataset!\n");
      return -1;
   }

   else {
      return inliers->indices.size();
   }

}


// Load an image to test plane segmentation code.
// The plane in this image can't currently be detected.
int load_image(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
   // Temporary location from file - TODO get a custom file
   char * cloud_file = (char *) "/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/data/saved_kinect_pcd.pcd";
   if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_file, *cloud) == -1)
   {
      return -1;
   } else {
      return 0;
   }
}

// Load a PCL file from a file.
// Usage: <this_executable> --file <filename>
int load_pcl_from_file(cloud_ptr cloud,
                       int argc,
                       char ** argv)
{
   string filename;

   // Check if the argument exists
   if (pcl::console::find_argument(argc, argv, "--file") < 0)
   {
      PCL_ERROR("Couldn't find a '--file' argument on the command line!\n");
      return -1;
   }

   // Parse the argument
   std::stringstream ss_filename(filename);
   int rval = pcl::console::parse_argument(argc, argv, "--file", filename);
   std::cout << "Attemting to read in file [" << ss_filename << "]" << std::endl;

   if (pcl::io::loadPCDFile(filename, *cloud) == -1)
   {
      PCL_ERROR("Couldn't read file!\n");
      return -1;
   } else {
      return 0;
   }
}

// Constructs a viewer with some basic parameters.
pcl::visualization::PCLVisualizer * construct_viewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
   pcl::visualization::PCLVisualizer * viewer = new pcl::visualization::PCLVisualizer("3D Viewer");

   viewer->setBackgroundColor(0, 0, 0);
   viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

   return viewer;
}

void
remove_from_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, PointIndices::Ptr indices)
{
   // Remove the entries at 'indices' in cloud.
}

int main (int argc, char ** argv)
{
   // Main method - load the .pcd file we want, and put it up on the visualizer.
  
// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));


   // Declare a point cloud object, read in data from file.
   pcl::PointCloud<pcl::PointXYZ>::Ptr my_cloud (new pcl::PointCloud<pcl::PointXYZ>);

   if (load_pcl_from_file(my_cloud, argc, argv) != 0)
   {
      PCL_ERROR("There was an error loading an input .pcd file.\n");
      return -1;
   }


   // Create a 'viewer' object
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (construct_viewer(my_cloud));

   // Create coefficients objects
   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

   // If --segment option is not present on command line, end things here
   if (pcl::console::find_argument(argc, argv, "--segment") < 0) {
      while (!viewer->wasStopped())
      {
         viewer->spinOnce(100);
         boost::this_thread::sleep(boost::posix_time::microseconds(100000));
      }
      return 0;
   }

   prompt("Currently viewing captured point cloud. Next: segment to find plane in point cloud.");


   // Create segmentation object
   pcl::SACSegmentation<pcl::PointXYZ> segmentation;

   // Optional? What does this do?
   segmentation.setOptimizeCoefficients(true);
   segmentation.setModelType(pcl::SACMODEL_PLANE);
// segmentation.setMethodType(pcl::SAC_RANSAC);

   // ????? How bad is this going to hurt the code
   segmentation.setDistanceThreshold(3);

   // A distance value of '3' seems to get the job done
   // e.g. segmentation.setDistanceThreshold(3)

   // Inliers??
   PointIndices::Ptr inliers (new PointIndices);
   segmentation.setInputCloud(my_cloud);

   // Inliers: resultant point indices that support the model found

   // void pcl::SACSegmentation<PointT>::segment(PointIndices & inliers, ModelCoefficients & model_coefficients)
   // inliers - resultant point indices that support the model found
   // model_coefficients - the resultant model coefficients
   // Looks like this fills 'inliers' and 'coefficients'
   segmentation.segment(*inliers, *coefficients );

   if (inliers->indices.size() == 0)
   {
      PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
      return (-1);
   } 

   std::cerr << "Model Coefficients: " << coefficients->values[0] << " "
      << coefficients->values[1] << " "
      << coefficients->values[2] << " "
      << coefficients->values[3] << std::endl;

   std::cerr << "Number of inliners: " << inliers->indices.size() << std::endl;
   std::cerr << "Total point cloud size: " << my_cloud->size() << std::endl;

   prompt("Press key to display plane.");

   // Display the plane..
   viewer->addPlane(*coefficients, "plane");
         
   while (!viewer->wasStopped())
   {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
   }

   prompt("Press key to remove inliers from original point cloud.");

   remove_from_point_cloud(my_cloud, inliers);

   return 0;
}
