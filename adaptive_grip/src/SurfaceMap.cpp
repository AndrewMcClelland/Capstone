#include "SurfaceMap.h"


int
SurfaceMap::convex_hull()
{
	int n = coordinates.size();
	int k = 0;

	hull.resize(n*2);

	// Sort points lexicographically - works because we defined "_Coordinate::operator <"
	sort(coordinates.begin(), coordinates.end());

	// Build lower hull
	for (int i = 0; i < n; ++i)
	{

		while (k >= 2 && cross(hull[k-2], hull[k-1], coordinates[i]) <= 0) k--;
		hull[k++] = coordinates[i];
	}

	// Build upper hull
	for (int i = (n - 2), t = (k+1); i >= 0; i--) {
		while (k >= t && cross(hull[k-2], hull[k-1], coordinates[i]) <= 0) k--;
		hull[k++] = coordinates[i];
	}

	// k-1 is the right size
	hull.resize(k-1); // flag -- alternatively write everything to -1 
	// and get the first nz index

///return hull.size();

	typedef boost::geometry::model::d2::point_xy<double>	Point;
    typedef boost::geometry::model::polygon<Point>			Polygon;

	Polygon polygon;

	/* Populate the polygon */

    std::vector<Point> poly_points;
    typedef std::vector<Coordinate>::const_iterator CoordIt;
    for (CoordIt it = hull.begin(); it != hull.end(); ++it)
    {
        poly_points.push_back(Point(it->x, it->y));
    }
    boost::geometry::assign_points(polygon, poly_points);
	boost::geometry::model::box<Point>	box;
	boost::geometry::envelope(polygon, box);

	std::cout << "Envelope: " << boost::geometry::dsv(box) << std::endl;


	return hull.size();
}
// Use the GTE library to get er done


float 
SurfaceMap::cross(const Coordinate &O, const Coordinate &A, const Coordinate &B)
{
	return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

int 
SurfaceMap::initialize(PointCloud::Ptr surface, const Point & centroid, const Calibration & cal)
{
	typedef PointCloud::const_iterator		Iterator;
	Point 											difference;
	float												x_diff;
	float												y_diff;


	for (Iterator i = surface->begin(); i != surface->end(); ++i)
	{
		PCLUtils::subtractXYZ(*i, centroid, difference);

		x_diff = PCLUtils::dot_double_normalize(difference, cal.x_vector);
		y_diff = PCLUtils::dot_double_normalize(difference, cal.y_vector);

		coordinates.push_back(Coordinate(x_diff, y_diff));
	}

	convex_hull();
	calculateMinimumAreaRect();

	return SUCCESS;
}

void
SurfaceMap::calculateMinimumAreaRect()
{
	// use 'hull'
	minimumAreaRectangle.area = FLT_MAX;

	for (size_t i0 = hull.size() -1, i1 = 0; i1 < hull.size(); i0 = i1++)
	{
		Coordinate origin = hull[i0];
		Coordinate U0		= hull[i1] - origin;

		U0.Normalize(); // Length of U0 is 1

		Coordinate U1 =  U0.Perp(); // flag

		float min0 = 0, max0 = 0;
		float max1 = 0;

		float dot_test = U0.dot(U1);

        //std::cout << "dot_test = " << dot_test << std::endl;

///	std::cout << "------------------------ Next ------------------------" << std::endl;
		for (size_t j = 0; j < hull.size(); ++j)
		{
			Coordinate D = hull[j] - origin;
			float dot = U0.dot(D);
			const float min_dot = 0.001;

///		std::cout << "     U0.dot(D) = " << dot << std::endl;
			if (dot > min_dot || (-1.0 * dot) > min_dot) {
				if (dot < min0) {
					min0 = dot;
				} else if (dot > max0) {
					max0 = dot;
				} 
		   }

			dot = U1.dot(D);
			if (fabs(dot) > min_dot) {
				if (dot > max1) {
					max1 = dot;
				}
			}


		}
			float area = (max0 - min0) * max1;
			if (area != 0 && area < minimumAreaRectangle.area)
			{
				minimumAreaRectangle.center = origin + ((min0 + max0) / 2.0) * U0 + (max1/2.0) * U1;
				minimumAreaRectangle.axis[0] = U0;
				minimumAreaRectangle.axis[1] = U1;
				minimumAreaRectangle.extent[0] = (max0 - min0) / 2.0;
				minimumAreaRectangle.extent[1] = max1 / 2.0;
				minimumAreaRectangle.area = area;

	//////	 std::cout << "============================================================" << std::endl;
	//////	std::cout << "Got new minimum area: " << area << std::endl;
	//////	 std::cout << "extent[0] = " << minimumAreaRectangle.extent[0] << std::endl;
	//////	 std::cout << "extent[1] = " << minimumAreaRectangle.extent[1] << std::endl;
	//////	 std::cout << "min0      = " << min0 << std::endl;
	//////	 std::cout << "max0      = " << max0 << std::endl;
	//////	 std::cout << "Origin      : (" << origin.x << ", " << origin.y << ")" << std::endl;
	//////	 std::cout << "Dotted With : (" << D.x << ", " << D.y << ")" << std::endl;
	//////	 std::cout << "U1          : (" << U1.x << ", " << U1.y << ")" << std::endl;
	//////	 std::cout << "============================================================" << std::endl;

				 // Display the points that got min_dot and max_dot
			}
	}

///return minimumAreaRectangle;

}


