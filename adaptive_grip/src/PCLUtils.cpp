#include "PCLUtils.h"

//void PCLUtils::project_point_on_plane(
//   const PointT &p1,
//   const Eigen::Vector4f &plane,
//   PointT &projected 
//) {
//   
//   // Plane Normal : first 3 coefficients of the plane..
//   Eigen::Vector3f plane_normal(plane[0], plane[1], plane[2]);
//
//   // Point on the plane. 
//   Eigen::Vector3f plane_origin(0.0, 0.0, (plane[3]/plane[2]));
//
//   Eigen::Vector3f point_to_project(p1.x, p1.y, p1.z);
//   Eigen::Vector3f projected_point;
//
//   // Project the point
//   pcl::geometry::project(point_to_project, 
//           plane_origin,
//           plane_normal,
//           projected_point)
//
//   // Assign the projected point's co-ordinates
//   projected.x = projected_point[0];
//   projected.y = projected_point[1];
//   projected.z = projected_point[2];
//
//}
//

template <typename PointT>
float PCLUtils::angleBetween(const PointT & pt_one, const PointT & pt_two)
{
   Eigen::Vector3f vec_one(pt_one.x, pt_one.y, pt_one.z);
   Eigen::Vector3f vec_two(pt_two.x, pt_two.y, pt_two.z);

   float cosine = vec_one.dot(vec_two);
   cosine = cosine / vec_one.norm();
   cosine = cosine / vec_two.norm();

   return cosine;
}

// Template instantiations to speed up compile
template float PCLUtils::angleBetween<pcl::PointXYZRGBA>(const pcl::PointXYZRGBA & pt_one, const pcl::PointXYZRGBA & pt_two);
template float PCLUtils::angleBetween<pcl::PointXYZ>(const pcl::PointXYZ & pt_one, const pcl::PointXYZ & pt_two);


template <typename PointT>
void PCLUtils::project(const PointT & original, const PointT & target, PointT & projection)
{
   Eigen::Vector3f vec_original(original.x, original.y, original.z);
   Eigen::Vector3f vec_target(target.x, target.y, target.z);

   // (original [DOT] target) / norm(target)

   float dot = vec_original.dot(vec_target);
   Eigen::Vector3f proj = (vec_target * dot) / vec_target.norm();
   projection.x = proj(0);
   projection.y = proj(1);
   projection.z = proj(2);
}


template void PCLUtils::project<pcl::PointXYZRGBA>(const pcl::PointXYZRGBA & original, const pcl::PointXYZRGBA & target, pcl::PointXYZRGBA & projection);
template void PCLUtils::project<pcl::PointXYZ>(const pcl::PointXYZ & original, const pcl::PointXYZ & target, pcl::PointXYZ & projection);

template <typename PointT>
float PCLUtils::dot_double_normalize(const PointT & original, const PointT & target)
{
   Eigen::Vector3f vec_original(original.x, original.y, original.z);
   Eigen::Vector3f vec_target(target.x, target.y, target.z);
   float normalized =  (vec_original.dot(vec_target) / vec_target.norm());
   normalized = normalized / vec_target.norm();
   return normalized;
}

template float PCLUtils::dot_double_normalize<pcl::PointXYZRGBA>(const pcl::PointXYZRGBA & original, const pcl::PointXYZRGBA & target);
template float PCLUtils::dot_double_normalize<pcl::PointXYZ>(const pcl::PointXYZ & original, const pcl::PointXYZ & target);

template <typename PointT>
void PCLUtils::pairAlign (const typename pcl::PointCloud<PointT>::Ptr cloud_src, 
      const typename pcl::PointCloud<PointT>::Ptr cloud_tgt, 
      typename pcl::PointCloud<PointT>::Ptr output, 
      Eigen::Matrix4f &final_transform, 
      Logger * logger)
{

   typedef typename pcl::PointCloud<PointT> PointCloud;
   typedef pcl::PointNormal        PointNormalT;
   typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

   bool downsample = false;
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  typename PointCloud::Ptr src (new PointCloud);
  typename PointCloud::Ptr tgt (new PointCloud);

  pcl::VoxelGrid<PointT> grid;

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


  // Try a removeNanFromPointCloud
  // NTP You have to do this or else things get real fucky!!
  std::vector<int> indices1, indices2;
  pcl::removeNaNFromPointCloud(*src, *src, indices1);
  pcl::removeNaNFromPointCloud(*tgt, *tgt, indices2);

  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
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

    {
       std::stringstream ss;
       ss << "Iteration Nr. " << i;
       logger->log(ss);
    }

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
//  showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }

	//
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

//viewer->removePointCloud ("source_after");
//viewer->removePointCloud ("target_after");

//PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
//PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
//viewer->addPointCloud (output, cloud_tgt_h, "target_after", vp_after);
//viewer->addPointCloud (cloud_src, cloud_src_h, "source_after", vp_after);

// PCL_INFO ("Press q to continue the registration.\n");
//viewer->spin ();

//viewer->removePointCloud ("source"); 
//viewer->removePointCloud ("target");

  //add the source to the transformed target
  *output += *cloud_src;
  
  final_transform = targetToSource;

  // Now take apart final_transform
  //
  {
     std::stringstream ss;
     ss << final_transform; // Let's see..
     logger->log("Transform Matrix: ");
     logger->log(ss);
  }

  Eigen::Affine3f tmp;
          tmp.matrix() = final_transform.matrix();

          Eigen::MatrixXf 	translation = tmp.translation();
          Eigen::MatrixXf 	rotation	= tmp.rotation();
          Eigen::MatrixXf	scaling		= tmp.linear();

  {
      std::stringstream ss;
      ss << "Translation Matrix: " << std::endl;
      ss << translation << std::endl;

      ss << "Rotation Matrix: " << std::endl;
      ss << rotation << std::endl;

      ss << "Scaling Matrix: " << std::endl;
      ss << scaling << std::endl;

      logger->log(ss);
  }

  
 }


template 
void PCLUtils::pairAlign<pcl::PointXYZRGBA>(const typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_src, 
      const typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tgt, 
      typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output, 
      Eigen::Matrix4f &final_transform, 
      Logger * logger);

template <typename PointT>
void PCLUtils::subtractXYZ(const PointT & _this_, const PointT & _minus_this_, PointT & _equals_this_)
{
   _equals_this_.x = _this_.x - _minus_this_.x;
   _equals_this_.y = _this_.y - _minus_this_.y;
   _equals_this_.z = _this_.z - _minus_this_.z;
}
template <typename PointT>
void PCLUtils::addXYZ(const PointT & _this_, const PointT & _plus_this_, PointT & _equals_this_)
   {
      _equals_this_.x = _this_.x + _plus_this_.x;
      _equals_this_.y = _this_.y + _plus_this_.y;
      _equals_this_.z = _this_.z + _plus_this_.z;
   }
template <typename PointT>
void PCLUtils::multiplyXYZ(const PointT & _this_, float _times_this_, PointT & _equals_this_)
   {
      _equals_this_.x = _this_.x * _times_this_;
      _equals_this_.y = _this_.y * _times_this_;
      _equals_this_.z = _this_.z * _times_this_;
   }

template void PCLUtils::subtractXYZ<pcl::PointXYZ>(const pcl::PointXYZ &, const pcl::PointXYZ &, pcl::PointXYZ &);
template void PCLUtils::subtractXYZ<pcl::PointXYZRGBA>(const pcl::PointXYZRGBA &, const pcl::PointXYZRGBA &, pcl::PointXYZRGBA &);
template void PCLUtils::addXYZ<pcl::PointXYZ>(const pcl::PointXYZ &, const pcl::PointXYZ &, pcl::PointXYZ &);
template void PCLUtils::addXYZ<pcl::PointXYZRGBA>(const pcl::PointXYZRGBA &, const pcl::PointXYZRGBA &, pcl::PointXYZRGBA &);
template void PCLUtils::multiplyXYZ<pcl::PointXYZ>(const pcl::PointXYZ &, float, pcl::PointXYZ &);
template void PCLUtils::multiplyXYZ<pcl::PointXYZRGBA>(const pcl::PointXYZRGBA &, float, pcl::PointXYZRGBA &);

template <typename PointT>
float PCLUtils::distance(const PointT & one, const PointT& two)
{
   PointT difference;
   subtractXYZ(one, two, difference);
   Eigen::Vector3f difference_vec(difference.x, difference.y, difference.z);
   return difference_vec.norm();
}

template float PCLUtils::distance<pcl::PointXYZ>(const pcl::PointXYZ &, const pcl::PointXYZ &);
template float PCLUtils::distance<pcl::PointXYZRGBA>(const pcl::PointXYZRGBA &, const pcl::PointXYZRGBA &);


void PCLUtils::dbWaitForEnter(std::string msg)
{
   char Enter = 0x0a;
   char c = 0;
   std::cout << "[DEBUG] " << msg << std::endl;
   std::cout << "[Press Enter to Continue]" << std::endl;
   do std::cin.get(c); while (c != Enter);
}

template <typename PointT> void PCLUtils::downsample(typename pcl::PointCloud<PointT>::Ptr	cloud, float voxelSize)
{
	// Debug information...
    std::cout << "PCLUtils::downsample() - size before downsampling = " << cloud->size() << std::endl;

    pcl::VoxelGrid<PointT> voxel;
	voxel.setInputCloud(cloud);
    voxel.setLeafSize(voxelSize, voxelSize, voxelSize);
	voxel.filter(*cloud);

    std::cout << "PCLUtils::downsample() - size after downsampling = " << cloud->size() << std::endl;
}

template void PCLUtils::downsample<pcl::PointXYZ>(pcl::PointCloud<pcl::PointXYZ>::Ptr, float);
template void PCLUtils::downsample<pcl::PointXYZRGBA>(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, float);
