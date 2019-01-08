#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

//#include <pcl/visualization/pcl_visualizer.h>
#include <inst/pcl_visualizer.h>

#include "Mutex.h"
#include "common_defs.h"
#include "TwoPlaneFilter.h"

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
typedef pcl::PointXYZ point_t;
typedef pcl::PointCloud<point_t> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
//using namespace std;

pcl::visualization::PCLVisualizer   *p;
int                                 vp_1, vp_2;


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature std::vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};


void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");

  PointCloudColorHandlerCustom<point_t> tgt_h (cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<point_t> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

  PCL_INFO ("Press q to begin the registration.\n");
  p-> spin();
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
  p->removePointCloud ("source");
  p->removePointCloud ("target");


  PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
  if (!tgt_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");

  PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
  if (!src_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");


  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
  p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

  p->spinOnce();
}

void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<point_t> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<point_t, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  const char b = 'a';
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);  
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < 60; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
    prev = reg.getLastIncrementalTransformation ();

    // visualize current state
    showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }

	//
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  p->removePointCloud ("source");
  p->removePointCloud ("target");

  PointCloudColorHandlerCustom<point_t> cloud_tgt_h (output, 0, 255, 0);
  PointCloudColorHandlerCustom<point_t> cloud_src_h (cloud_src, 255, 0, 0);
  p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
  p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

	PCL_INFO ("Press q to continue the registration.\n");
  p->spin ();

  p->removePointCloud ("source"); 
  p->removePointCloud ("target");

  //add the source to the transformed target
  *output += *cloud_src;
  
  final_transform = targetToSource;

  
 }

int main (int argc, char * argv[])
{
   std::string filename_one;
   std::string filename_two;
   
   // Load point clouds
   if (argc < 3) {
      cerr << "Not enough arguments provided. Exiting." << std::endl;
      filename_one = "/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/data/registration/ball_one.pcd";
      filename_two = "/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/data/registration/ball_two.pcd";
//    return FAILURE;
   } else {
      filename_one = argv[1];
      filename_two = argv[2];
   }

   // Get Filenames.

   std::cout << "First file: " << filename_one << std::endl;
   std::cout << "Second file: " << filename_two << std::endl;
   std::cout << "argc : " << argc << std::endl;


   PointCloud::Ptr cloud_src(new PointCloud());
   PointCloud::Ptr cloud_tgt(new PointCloud());
   Mutex * mutex = new Mutex();


   TwoPlaneFilter<point_t>   filt_src(cloud_src, mutex, 0.1);
   TwoPlaneFilter<point_t>   filt_tgt(cloud_tgt, mutex, 0.1);


   // Before we start, get rid of the claw's points.
   

   // loadPCDFile() calls.

   if ( (pcl::io::loadPCDFile(filename_one, *cloud_src) != 0) || 
        (pcl::io::loadPCDFile(filename_two, *cloud_tgt) != 0))
   {
      cerr << "There was a problem loading one or both of the point clouds." << std::endl;
      return FAILURE;
   }

   // Remove planes
   filt_src.filter_plane();
   filt_tgt.filter_plane();
   
   // RemoveNaN
      // From cloud_src
   std::vector<int> indices;
   pcl::removeNaNFromPointCloud(*cloud_src, *cloud_src, indices);
   std::cout << "Removed " << indices.size() << " indices from cloud_src." << std::endl;

      // From cloud_tgt
   indices.clear();
   pcl::removeNaNFromPointCloud(*cloud_tgt, *cloud_tgt, indices);
   std::cout << "Removed " << indices.size() << " indices from cloud_tgt." << std::endl;

   p = new pcl::visualization::PCLVisualizer(argc, argv, "Viewer");
   p->createViewPort(0.0, 0, 0.5, 1.0, vp_1);
   p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);

   PointCloud::Ptr result (new PointCloud), source, target;
   Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

   showCloudsLeft(cloud_src, cloud_tgt);

   PointCloud::Ptr temp (new PointCloud);
   PCL_INFO("Aligning..");
   pairAlign(cloud_src, cloud_tgt, temp, pairTransform, true);

   pcl::transformPointCloud(*temp, *result, GlobalTransform);

   GlobalTransform = GlobalTransform * pairTransform;

// std::cout << "GlobalTransform[0] = " << GlobalTransform[0] << std::endl;
// std::cout << "GlobalTransform[1] = " << GlobalTransform[1] << std::endl;
// std::cout << "GlobalTransform[2] = " << GlobalTransform[2] << std::endl;
// std::cout << "GlobalTransform[3] = " << GlobalTransform[3] << std::endl;

   PCL_INFO("Complete.");

   while (!p->wasStopped()) {
      p->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
   }

}
