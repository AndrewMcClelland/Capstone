#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/io.h>

using namespace pcl;
  typedef PointXYZRGB RGBPoint;
//typedef PointXYZ RGBPoint;

int
main (int argc, char** argv)
{
   // Declare Point Cloud (cloud_blob), Filtered Point Cloud (cloud_filtered, cloud_p, cloud_f)
   pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
   pcl::PointCloud<PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<PointXYZ>), cloud_p (new pcl::PointCloud<PointXYZ>), cloud_f (new pcl::PointCloud<PointXYZ>);
 
   // Fill in the cloud data
   pcl::PCDReader reader;
   reader.read ("../table_scene_lms400.pcd", *cloud_blob);
 
   std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
 
   // Create the filtering object: downsample the dataset using a leaf size of 1cm
   pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
   sor.setInputCloud (cloud_blob);
   sor.setLeafSize (0.01f, 0.01f, 0.01f);
   sor.filter (*cloud_filtered_blob);
 
   // Convert to the templated PointCloud
   pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
 
   std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
 
   // Write the downsampled version to disk
   pcl::PCDWriter writer;
   writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);
 
   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
   pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
   // Create the segmentation object
   pcl::SACSegmentation<pcl::PointXYZ> seg;
   // Optional
   seg.setOptimizeCoefficients (true);
   // Mandatory
   seg.setModelType (pcl::SACMODEL_PLANE);
   seg.setMethodType (pcl::SAC_RANSAC);
   seg.setMaxIterations (1000);
   seg.setDistanceThreshold (0.01);
 
   // Create the filtering object
   pcl::ExtractIndices<pcl::PointXYZ> extract;
 
   int i = 0, nr_points = (int) cloud_filtered->points.size ();

   pcl::PointCloud<RGBPoint>::Ptr coloured_cloud (new pcl::PointCloud<RGBPoint>);

   // While 30% of the original cloud is still there
   while (cloud_filtered->points.size () > 0.3 * nr_points)
   {
     // Segment the largest planar component from the remaining cloud
     seg.setInputCloud (cloud_filtered);
     seg.segment (*inliers, *coefficients);
     if (inliers->indices.size () == 0)
     {
       std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
       break;
     }
 
     // Extract the inliers
     extract.setInputCloud (cloud_filtered);
     extract.setIndices (inliers);
     extract.setNegative (false);
     extract.filter (*cloud_p);
     std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
 
     
     // "Inliers" contains the inliers of all points in the current plane
     //
     int j = 0;


     pcl::PointCloud<PointXYZ> temp_cloud_xyz;
     pcl::PointCloud<RGBPoint> temp_cloud_xyzrgb;

     // Copy RGB points in indices of clould_filtered into temp_cloud_xyz
     for (std::vector<int>::const_iterator it = inliers->indices.begin(); it != inliers->indices.end(); ++it)
     {
         temp_cloud_xyz.points.push_back(cloud_filtered->points[*it]);
     }
     // Copy the XYZ Point Cloud into an XYZRGB Point Cloud
     pcl::copyPointCloud(temp_cloud_xyz, temp_cloud_xyzrgb);

     // TODO ALSO TODO See: pcl::copyPointCLoud(cloud_in, indices, cloud_out)
     
     // Add the XYZRGB Point Cloud to coloured_cloud
     (*coloured_cloud) += temp_cloud_xyzrgb;
     
     std::cout << "Size of `coloured_cloud`: " << coloured_cloud->points.size() << std::endl;

     std::stringstream ss;
     ss << "table_scene_lms400_plane_" << i << ".pcd";
     writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
 
     // Create the filtering object
     extract.setNegative (true);
     extract.filter (*cloud_f);
     cloud_filtered.swap (cloud_f);
     i++;
   }


   uint8_t r = 255, g = 0, b = 0;
   uint32_t red = ((uint32_t) 4 << 16 | (uint32_t) g << 8 | (uint32_t) b);
   // Set colours for all of the points in coloured_cloud...
   for (pcl::PointCloud<RGBPoint>::iterator it = coloured_cloud->begin(); it != coloured_cloud->end(); ++it)
   {
      // FTR this is how to change point cloud RGB values
       (*it).r = 255;
       (*it).g = 0;
       (*it).b = 0;
   }

   std::cout << "Size of `coloured_cloud` (outside of loop): " << coloured_cloud->points.size() << std::endl;

   // New: add a visualizer with the filtered cloud, that should have planar points coloured
   pcl::visualization::PCLVisualizer * viewer = new pcl::visualization::PCLVisualizer("Coloured Points");
   viewer->setBackgroundColor(0, 0, 0);
   viewer->addPointCloud<RGBPoint>(coloured_cloud, "Planes");
   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Planes");

   while (!viewer->wasStopped())
   {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
   }

   return (0);
}
