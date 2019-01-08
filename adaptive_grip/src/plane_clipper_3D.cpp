#include <inst/plane_clipper_3D.h>
#include <pcl/point_types.h>

#include <pcl/filters/impl/plane_clipper3D.hpp>

template class pcl::PlaneClipper3D<pcl::PointXYZ>;
template class pcl::PlaneClipper3D<pcl::PointXYZRGBA>;

