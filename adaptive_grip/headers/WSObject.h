#ifndef WSOBJECT__H
#define WSOBJECT__H

#include <Eigen/Dense>
#include <pcl/point_types.h>

class WSObject {

   public:

   float          x_position;
   float          y_position;

   // Don't store any z-axis information or "surface" information in 
   // WSObject. Handle it in Engine2.
// float          z_height;

   float          observation_distance;

   pcl::PointXYZRGBA   point;

   int            id;
// float          distance_to(WSObject & o, Eigen::Vector4f & plane);

   static float   fudge;
   static float   distance_fudge;
   static int     current_id;

	float x_obs_position, y_obs_position;
	float r_display, g_display, b_display;
	float plane_distance;

   WSObject(float x, float y, float obs_dist);

   // Copy constructor
   WSObject(const WSObject & copy); 

	bool operator== (const WSObject& ws) const;

};

#endif 
