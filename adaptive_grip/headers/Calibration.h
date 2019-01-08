#ifndef CALIBRATION__H
#define CALIBRATION__H

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <pcl/point_representation.h>
#include <pcl/point_cloud.h>

class Calibration
{
public:

   typedef  pcl::PointXYZRGBA       Point;
// typedef  pcl::PointCloud<Point>  PointCloud;

   // Calibration
   float                            x_rpos_amt;
   float                            y_rpos_amt;
   Point                            x_vector;
   Point                            y_vector;

   float                            x_adj_amt;
   float                            y_adj_amt;
   
   bool toFile(const std::string& filename);

   bool fromFile(const std::string& filename);

   std::string toString();

}; 

#endif
