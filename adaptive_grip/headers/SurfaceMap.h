#ifndef SURFACE_MAP__H
#define SURFACE_MAP__H

#include "common_defs.h"
#include "PCLUtils.h"
#include "Calibration.h"
#include "Coordinate.h"
#include <algorithm>
#include <vector>
#include <limits>

#ifdef GTE
#include <Mathematics/GteBSRational.h>
#include <Mathematics/GteMinimumAreaBox2.h>
//#include <Mathematics/GteUIntegerAP32.h>
#endif

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

//#define X 0
//#define Y 1

class SurfaceMap {

public:



	typedef struct {
		Coordinate center;
		Coordinate axis[2];
		float 	  extent[2];
		float 	  area;

		std::vector<Coordinate> 	corners() {
			std::vector<Coordinate>	cornerVect;
			Coordinate cornerOne 		= center + extent[0] * axis[0] + extent[1] * axis[1];
			Coordinate cornerTwo 		= center - extent[0] * axis[0] + extent[1] * axis[1];
			Coordinate cornerThree 		= center + extent[0] * axis[0] - extent[1] * axis[1];
			Coordinate cornerFour 		= center - extent[0] * axis[0] - extent[1] * axis[1];

			cornerVect.push_back(cornerOne);
			cornerVect.push_back(cornerTwo);
			cornerVect.push_back(cornerThree);
			cornerVect.push_back(cornerFour);

			return cornerVect;


		}
	} Rectangle;


	// 2D cross product of OA and OB vectors.
	float cross(const Coordinate &O, const Coordinate &A, const Coordinate &B);

	// Returns a vector of points forming a convex hull.
	int	convex_hull();
	int	convex_hull_2();

	Rectangle minimumAreaRectangle;
	void calculateMinimumAreaRect();

	std::vector<Coordinate> coordinates;
	std::vector<Coordinate> hull;
	

	typedef pcl::PointXYZRGBA 									Point;
	typedef pcl::PointCloud<Point>							PointCloud;

	// Build a surface map from a Kinect scan.
	int initialize(PointCloud::Ptr surface, const Point & centroid, const Calibration & cal);


};


#endif
