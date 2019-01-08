#include <pcl/point_types.h>

#include <inst/pcl_visualizer.h>

#include <pcl/visualization/impl/pcl_visualizer.hpp>

/* 
 * Implicit instantiations of pcl::visualization::PCLVisualizer
 * functions so we don't have to compile them any time a file
 * that #includes <pcl/visualization/pcl_visualizer.h> is changed.
 * 
 * It was hurting compile time so much that it validated explicitly
 * instantiating these functions here.
 */


template  bool
pcl::visualization::PCLVisualizer::addPointCloud<pcl::PointNormal> (
  const typename pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud,
  const PointCloudColorHandler<pcl::PointNormal> &color_handler,
  const std::string &id, int viewport);

template bool
pcl::visualization::PCLVisualizer::addPointCloud<pcl::PointXYZ> (
  const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
  const PointCloudColorHandler<pcl::PointXYZ> &color_handler,
  const std::string &id, int viewport);

template bool
pcl::visualization::PCLVisualizer::addPointCloud<pcl::PointXYZRGBA> (
  const typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud,
  const PointCloudColorHandler<pcl::PointXYZRGBA> &color_handler,
  const std::string &id, int viewport);

template bool
pcl::visualization::PCLVisualizer::addPointCloud<pcl::PointXYZ> (
  const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
  const std::string &id, int viewport);

template bool
pcl::visualization::PCLVisualizer::addPointCloud<pcl::PointXYZRGBA> (
  const typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud,
  const std::string &id, int viewport);

template bool
pcl::visualization::PCLVisualizer::updatePointCloud<pcl::PointXYZ> (const typename pcl::PointCloud<PointXYZ>::ConstPtr &cloud,
                                                     const std::string &id);
template bool
pcl::visualization::PCLVisualizer::updatePointCloud<pcl::PointXYZRGBA> (const typename pcl::PointCloud<PointXYZRGBA>::ConstPtr &cloud,
                                                     const std::string &id);

template bool pcl::visualization::PCLVisualizer::updatePointCloud<pcl::PointXYZRGBA>(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr const&, pcl::visualization::PointCloudColorHandler<pcl::PointXYZRGBA> const&, std::basic_string<char, std::char_traits<char>, std::allocator<char> > const&);
template bool pcl::visualization::PCLVisualizer::updatePointCloud<pcl::PointXYZ>(pcl::PointCloud<pcl::PointXYZ>::ConstPtr const&, pcl::visualization::PointCloudColorHandler<pcl::PointXYZ> const&, std::basic_string<char, std::char_traits<char>, std::allocator<char> > const&);

template bool pcl::visualization::PCLVisualizer::addLine<pcl::PointXYZRGBA, pcl::PointXYZRGBA>(pcl::PointXYZRGBA const&, pcl::PointXYZRGBA const&, std::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, int);
template bool pcl::visualization::PCLVisualizer::addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl::PointXYZ const&, pcl::PointXYZ const&, std::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, int);

template bool pcl::visualization::PCLVisualizer::addLine<pcl::PointXYZRGBA, pcl::PointXYZRGBA>(pcl::PointXYZRGBA const&, pcl::PointXYZRGBA const&, double, double, double, std::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, int);
template bool pcl::visualization::PCLVisualizer::addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl::PointXYZ const&, pcl::PointXYZ const&, double, double, double, std::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, int);

template bool pcl::visualization::PCLVisualizer::addLine<pcl::PointXYZRGBA, pcl::PointXYZ>(pcl::PointXYZRGBA const&, pcl::PointXYZ const&, std::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, int);
