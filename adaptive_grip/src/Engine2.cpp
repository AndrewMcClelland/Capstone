
#include "Engine2.h"

using std::stringstream;
using pcl::visualization::PointCloudColorHandlerCustom;
using PCLUtils::addXYZ;
using PCLUtils::subtractXYZ;
using PCLUtils::multiplyXYZ;
using std::endl;

int Engine2::add_object(Point point, bool closest)
{
   RobotPosition objectPosition = getPosition();
   calculate_robot_position(point, objectPosition);

   const float x_delta = objectPosition.x - getPosition().x;
   const float y_delta = objectPosition.y - getPosition().y;
   const float observation_distance = sqrt(x_delta*x_delta + y_delta*y_delta);


   typedef std::vector<WSObject>::iterator   Iterator;

   bool object_exists = false;

   // If we're too far away from an object to do anything, leave it
   if (observation_distance > WSObject::distance_fudge) {
      std::stringstream msg;
      msg << "add_object() : Ignoring the add at (" << objectPosition.x << ", " << objectPosition.y << "). Distance (" << observation_distance
         << ") above fudge factor (" << WSObject::distance_fudge << ")";
      m_logger->log(msg);
   }

   // Else, iterate through the existing objects.
   Iterator it;
   for (it = m_objects->begin(); it != m_objects->end(); it++) {
      
      float x_diff = (*it).x_position - objectPosition.x;
      float y_diff = (*it).y_position - objectPosition.y;
      float distance = sqrt((x_diff*x_diff) + (y_diff*y_diff));

      if ( distance < WSObject::fudge ) {
         std::stringstream ss;
         ss << "add_object(" << objectPosition.x << ", " << objectPosition.y << ") No add - there is an object at (" << (*it).x_position << "," << (*it).y_position << ")." << endl;
         ss << "distance = " << distance << ", fudge = " << WSObject::fudge << ".";              
         m_logger->log(ss);
         break;
      }
   }

   WSObject * update_object;
   WSObject * new_obj = NULL;

   if (it != m_objects->end()) { // We found the object already
      update_object = &(*it);
   } else { // We didnt
      std::stringstream ss;
      ss << "Added object at (" << objectPosition.x << ", " << objectPosition.y << ").";
      m_logger->log(ss);
      new_obj = new WSObject(objectPosition.x, objectPosition.y, observation_distance);
      update_object = new_obj;
   } 

   update_object->point = point;
   update_object->observation_distance = observation_distance;

   Cluster<Point> point_cluster(update_object->point);
   float distance_to_plane = fabs(point_cluster.get_distance_to_plane(m_floor_plane));
	update_object->plane_distance = (distance_to_plane + DEFAULT_FLOOR_HEIGHT);
   {
//    std::stringstream msg;
//    msg << "Calculated a distance-to-plane of " << (distance_to_plane + DEFAULT_FLOOR_HEIGHT)  << ".";
//    m_logger->log(msg);
   }

   // Update R, G, B
   if (closest) {
      update_object->r_display = 0;
      update_object->g_display = 255;
      update_object->b_display = 255;
   } else {
      update_object->r_display = 0;
      update_object->g_display = 255;
      update_object->b_display = 0;
   }

   if (new_obj) {

      m_objects->push_back(*update_object);
      delete new_obj;
   }

   return true;
}

bool Engine2::remove_object(float x, float y) {
   m_logger->log("remove_object not implemented yet.");
   return false;
}


RobotPosition  Engine2::getPosition()
{
// return m_robot->currentPos();
	return m_robot->currentPos();

// if (!position_valid) {
//    currentPosition = m_robot->currentPos();
// }
// position_valid = true;
// return currentPosition;
}


bool Engine2::validate_limits(const RobotPosition & pos) 
{
	// Check if the position we're validating is within
	// the robot's limits.
   #ifndef NO_ENGINE
	return m_robot->currentLimits().posWithin(pos);
   #else
    return false;
#endif
}

bool Engine2::calculate_camera_position(const Point & position, RobotPosition & new_pos)
{
   Point vector_along_floor;

   PCLUtils::subtractXYZ(position, origin_, vector_along_floor);

   float x_amt = PCLUtils::dot_double_normalize(vector_along_floor, m_cal.x_vector);
   float y_amt = PCLUtils::dot_double_normalize(vector_along_floor, m_cal.y_vector);

   new_pos.x += (-1.0 * x_amt * m_cal.x_rpos_amt);
   new_pos.y += (-1.0 * y_amt * m_cal.y_rpos_amt);

   return validate_limits(new_pos);
}

bool Engine2::calculate_robot_position(const Point & position, RobotPosition & new_pos)
{
   Point vector_along_floor;

   /*
    * Calculate the X and Y RobotPosition of the observed point
    */

   PCLUtils::subtractXYZ(position, get_claw_center_projection(), vector_along_floor);

   float x_amt = PCLUtils::dot_double_normalize(vector_along_floor, m_cal.x_vector);
   float y_amt = PCLUtils::dot_double_normalize(vector_along_floor, m_cal.y_vector);

   // what the fuck??? why did I have to change the magnitude of new_pos.y???
   // everything went to shit when I started running this code on the lab machine,
   // but it was working with the opposite magnitude (e.g. above on my mac)
   new_pos.x += (-1.0 * x_amt * m_cal.x_rpos_amt);
   new_pos.y += ( 1.0 * y_amt * m_cal.y_rpos_amt);

   /*
    * Calculate its "Z" value. Get its distance from the plane.
    */

   Cluster<Point> point_cluster(position);
   float distance_to_plane = fabs(point_cluster.get_distance_to_plane(m_floor_plane));
   {
      std::stringstream msg;
      msg << "Calculated a distance-to-plane of " << (distance_to_plane + DEFAULT_FLOOR_HEIGHT)  << ".";
      m_logger->log(msg);
   }

   return validate_limits(new_pos);
}

void Engine2::calibrate()
{
   // Calibrate the X-axis, if necessary
   if (!m_state.state.x_axis_calibrated) {
      m_state.state.x_axis_calibrated = calibrate_axis(true, m_cal.x_rpos_amt, m_cal.x_vector);
      if (m_state.state.x_axis_calibrated) {
         m_logger->log("Successfully calibrated the x-axis.");
      }
   } else m_logger->log("The x-axis was already calibrated.");


   // Calibrate the Y-axis, if necessary
   if (!m_state.state.y_axis_calibrated) {
      m_state.state.y_axis_calibrated = calibrate_axis(false, m_cal.y_rpos_amt, m_cal.y_vector);
      if (m_state.state.y_axis_calibrated) {
         m_logger->log("Successfully calibrated the y-axis.");
      }
   } else m_logger->log("The y-axis was already calibrated.");


   // If both axes are calibrated, set up the viewer
   if (m_state.state.x_axis_calibrated && m_state.state.y_axis_calibrated)
   {
      m_logger->log("Displaying calibration vectors.");
      axis_view_setup(vp_calibration_axes);
   }
}

void Engine2::load()
{
    using std::cout;
   const std::string calFile = "/home/robot/Documents/Group3/adaptive_grip/data/calibration.dat";
   if (m_cal.fromFile(calFile)) {
      cout << "Succesfully loaded file." << endl;
      cout << m_cal.toString() << endl;
      m_state.state.x_axis_calibrated = true;
      m_state.state.y_axis_calibrated = true;
      axis_view_setup(vp_calibration_axes);

      // TODO MOVE
      setup_claw_line(vp_calibration_axes);

   } else {
      std::cerr << "There was a problem loading the calibration file." << endl;
   }

   if (!m_got_floor_plane) {
      ClusterCloud target; // dummy ClusterCloud to get floor coefficients
      load_and_filter(target);
   }

   // Set up the claw line.
   move_claw_line(m_cal.x_adj_amt, m_cal.y_adj_amt, 0);
}

bool Engine2::calibrate_axis(bool x_yb, float & robot_pos_amt, Point & vector)
{

   robot_pos_amt = 100.0;

   ClusterCloud   source, target;

   RobotPosition new_pos = getPosition();
   if (x_yb) {
      new_pos.x = new_pos.x + robot_pos_amt;
   } else {
      new_pos.y = new_pos.y + robot_pos_amt;
   }

   if (!validate_limits(new_pos)) {
      stringstream ss("");
      ss << "calibrate_axis() : Illegal robot position while trying to calibrate.";
      m_logger->log(ss);
      return false;
   }

   // Load source cloud
   load_and_filter(source);
//    add_cloud_to_viewer(source.cloud, "TargetCloud", vp_calibration_clouds, 255, 0, 0);


   // Move to position
   moveTo(new_pos);

   // Load target cloud
   load_and_filter(target);
//    add_cloud_to_viewer(target.cloud, "SourceCloud", vp_calibration_clouds, 0, 0, 255);

   // ClusterDifference
   if (!clusterDifference(source, target, vector)) {
      stringstream ss;
      ss << "calibrate_axis() : clusterDifference() failed. Could not calibrate ";
      ss << (x_yb ? "x" : "y") << " axis." << endl;
      m_logger->log(ss);
      return false;
   }

   return true;
}

// This is only used for calibration, maybe worth moving?
bool Engine2::clusterDifference(ClusterCloud & source, ClusterCloud & target, Point & vector)
{
   source.find_clusters();
   target.find_clusters();

   if (source.size() != 1) {
      stringstream ss;
      ss << "clusterDifference() : Source cloud had " << source.size() << " clusters." << endl;
      ss << "Returning false." << std::endl;
      m_logger->log(ss);
      return false;
   } else if (target.size() != 1) {
      stringstream ss;
      ss << "clusterDifference() : Target cloud had " << target.size() << " clusters." << endl;
      ss << "Returning false." << std::endl;
      m_logger->log(ss);
      return false;
   }

   Point source_cluster = source.clusters->points[0];
   Point target_cluster = target.clusters->points[0];
   PCLUtils::subtractXYZ(target_cluster, source_cluster, vector);

   return true;
}

int   Engine2::locate(bool observe_only) 
{
   m_logger->log("locate() DEPRECATED.");
   return -1;
}

// TODO also update GUI with our last scan location
int Engine2::scan(bool analyse_closest)
{
   // find clusters
   // rank them based on distance to origin
   // move to the closest one

   typedef std::vector<Point>::iterator   Iterator;
   Iterator                               closest_cluster;

   // Get camera input
   load_raw(current_view);

   // Put it on the viewer
   add_cloud_to_viewer(current_view.cloud, "CurrentCloud", vp_navigation);

   // Filter the floor out
   filter_floor(current_view);

   // TODO CHECK THAT WE ARE CALIBRATED!
   if (!m_state.state.x_axis_calibrated || !m_state.state.y_axis_calibrated) {
      m_logger->log("scan() - Not calibrated. Aborting.");
      return -1;
   }

   // Find clusters
   current_view.find_clusters();

   {
      stringstream ss;
      ss << "After find_clusters(): cloud size = " << current_view.cloud->size();
      m_logger->log(ss);
   }

   if (current_view.size() == 0) {
      m_logger->log("scan() - No clusters found.");
      return -1;
   }
   else {
      add_objects(current_view);



      return 0;
   }


	return 1;
}


void
Engine2::create_closest_surface_map(ClusterCloud current_view, std::vector<WSObject>::iterator object)
{
	// Find the closest cluster in the cloud.
	typedef std::vector<int>::const_iterator IndicesIt;
	typedef std::vector<Point>::iterator	  PointIterator;

	PointIterator		closest_object_centroid;
	const RobotPosition currentPos = getPosition();

	// Go through the clusters in the current view, and find the one that's closest
	// to the object we are trying to approach.
	float shortest_xy_dist = -1;
	RobotPosition calculatedPosition = getPosition();
	for (PointIterator point = current_view.cf->m_centroids.begin(); point != current_view.cf->m_centroids.end(); ++point)
	{
		calculatedPosition = currentPos;
		calculate_robot_position(*point, calculatedPosition);

		float x_delta = calculatedPosition.x - object->x_position;
		float y_delta = calculatedPosition.y - object->y_position;

		float distance = sqrt(x_delta * x_delta + y_delta * y_delta);

		if (shortest_xy_dist < 0 || distance < shortest_xy_dist)
		{
			shortest_xy_dist = distance;
			closest_object_centroid = point;
		}
	}

	// Take the index of the closest cluster in m_centroids
	int cluster_index = std::distance(current_view.cf->m_centroids.begin(), closest_object_centroid);

	typedef std::vector<int>::const_iterator IndicesIt;
	std::vector<int> & points_vector = current_view.cf->m_clusters[cluster_index].indices; // save typing

	closest_cluster_cloud.reset(new PointCloud());
	for (IndicesIt point = points_vector.begin(); point != points_vector.end(); point++)
	{
		closest_cluster_cloud->push_back((*current_view.cloud)[*point]);
	}

	add_cloud_to_viewer(closest_cluster_cloud, "ClosestCluster", vp_calibration_axes, 255, 0, 0);

	// Stop the memory leak
	if (surface != NULL) {
		delete surface;
	}

	surface = new SurfaceMap();
   surface->initialize(closest_cluster_cloud, *closest_object_centroid, m_cal);

	 // At this point we have a valid rectangle in 'Surface'
	 // Find the largest of 'extent' and 'extent2'.
/// using SurfaceMap::Rectangle;

	 SurfaceMap::Rectangle &	top_surface = surface->minimumAreaRectangle;

	 int longest_axis;

	 if (fabs(top_surface.extent[0]) < fabs(top_surface.extent[1])) {
	 	longest_axis = 0;
	 } else {
		longest_axis = 1;
	 }

	 // Get the angle of the longest axis with the x-axis, in degrees
	 float 	angle;
	 angle = 	atan( top_surface.axis[longest_axis].y / top_surface.axis[longest_axis].x );
	 angle = 	angle * 180.0 / PI;

	 // And print out this angle.
	 {
		 std::stringstream msg;
		 msg << "Axis Y = " << top_surface.axis[longest_axis].y << endl;
		 msg << "Axis X = " << top_surface.axis[longest_axis].x << endl;
		 msg << "Calculated an angle of : " << angle;
		 m_logger->log(msg);
	 }

	 // We want to move j4 to `angle` + 90 degrees.

     j4_pickup_angle = angle/* + 90.0*/;

	 // If j4 is outside the robot limits, correct it by adding plus or minus 180.
	 const RobotLimits limits = m_robot->currentLimits();
	 while (j4_pickup_angle > limits.max().j4) {
			j4_pickup_angle -= 180.0;
	 }
	 while (j4_pickup_angle < limits.min().j4) {
			j4_pickup_angle += 180.0;
	 }

	 {
		 std::stringstream msg;
		 msg << "Corrected j4 to: " << j4_pickup_angle << ".";
		 m_logger->log(msg);
	 }



}

// fucking broken!! 
std::vector<WSObject>::iterator Engine2::get_closest_object()
{
   typedef std::vector<WSObject>::iterator Iterator;
   Iterator    closest = m_objects->begin();
// float shortest_distance = xy_distance(, getPosition());
//

// Get claw projection
   Point claw_center = get_claw_center_projection();
   float shortest_distance = -1;
   for (Iterator i = (m_objects->begin() + 1); i != m_objects->end(); ++i)
   {
      Cluster<Point> current_point_cluster(i->point);
      Point floorPoint = current_point_cluster.get_plane_projection(m_floor_plane);

      float distance = PCLUtils::distance(floorPoint, claw_center);

      if (shortest_distance < 0.0 || distance < shortest_distance) {
         shortest_distance = distance;
         closest = i;
      }
   }
   return closest;
}

// Should accept a pointer to a WSObject instead
//bool Engine2::prepare_grab(Point &centroid)
//
//Use after a scan only
bool Engine2::vantage_point(std::vector<WSObject>::iterator object)
{
   {
      std::stringstream msg;
      msg << "vantage-pointing to Object [" << object->id << "].";
      m_logger->log(msg);
   }

   m_logger->log("Warning: moving to a hard-coded \"vantage\" point to re-scan.");
   RobotPosition initial = getPosition();
   initial.x = object->x_position + VANTAGE_X_OFFSET; // Empirically determined
   initial.y = object->y_position + VANTAGE_Y_OFFSET;
   initial.z = 200.0; // pretty high

   moveTo(initial);
   return true;
}

bool Engine2::pickup(std::vector<WSObject>::iterator obj)
{
	// TODO we should exit if we're not close enough to the object.
	// TODO should also exit if the object was observed at a far-away location.

	m_logger->log("Entering Engine2::pickup()");

   scan();

	// First: position j4.
	m_logger->log("Positioning j4.");

   RobotPosition obj_position = getPosition();
   if (!calculate_robot_position(obj->point, obj_position))
   {
      m_logger->log("pickup() - the calcualted RobotPosition is outside of bounds.");
      return false;
   } else {

		// Set j4 correctly.
		obj_position.j4 = j4_pickup_angle;

      moveTo(obj_position);
      //position_valid = false;

		stringstream msg;

		// Empirically determined equation for calculating the height we want.
	   float z_height = 105 + (obj->plane_distance - 0.349483) * 1000; // 1000 = millimetres per metre

		msg << "The recommended Z-axis height is " << z_height << " (plane_distance = " << obj->plane_distance << ")";
		m_logger->log(msg);

      obj_position.z = z_height;
      moveTo(obj_position);
   }
	m_logger->log("Exiting Engine2::pickup()");
   return true;
}

void Engine2::load_raw(ClusterCloud & cc) 
{
   cc.cloud.reset(new PointCloud);
// cc.reset();
   m_camera.retrieve();

	// HERE: emit the 'done with camera' signal.
	// That way we can tell the live viewer to re-start its interface,
	// which will re-start the live viewer. You can only have one 
	// interface running at a time.
	emit RestartLiveFeed();


   *cc.cloud = *cam_cloud_;
// pcl::copyPointCloud(*cam_cloud_, *cc.cloud);

	// TODO FOR NOW TODO Test this downsampling.
    PCLUtils::downsample<Point>(cc.cloud, 0.0075);

   // Don't filter it
   cc.initialized = true;

}

void Engine2::moveTo(const RobotPosition & position, bool directly, float overrideSpeed)
{
// m_robot->moveTo(position, 0.5);
   //m_robot->moveTo(position, 0.15);
   // TODO could emit something for Qt... but how (since this isn't a Q_OBJECT)

   position_valid = false;
	if (!directly) {

	float 			new_z = position.z;

	
	// Go straight up...
	RobotPosition risePosition = getPosition();
	risePosition.z = z_ceiling;
	m_robot->moveTo(risePosition, RISE_SPEED);

	// Move to the new X/Y/J4/J5/J6
	RobotPosition hoverPosition = position;
///risePosition = position;
///risePosition.z = z_ceiling;
///risePosition.j5 = 0.0;
	hoverPosition.z = z_ceiling;
	hoverPosition.j5 = 0.0;
	m_robot->moveTo(hoverPosition, XY_SPEED);

	// Move to the new Z.
	m_robot->moveTo(position, DESCENT_SPEED);

	} else {
		m_robot->moveTo(position, overrideSpeed);
	}

}

void Engine2::filter_floor(ClusterCloud & cc) 
{
   Mutex dumb_mutex; // TODO un-mutex this
   TwoPlaneFilter<Point>   tp(cc.cloud, &dumb_mutex);
   tp.filter_plane();

   if (!m_got_floor_plane) {
      m_logger->log("load_and_filter() - Updating floor coefficients.");

      m_floor_plane = tp.get_plane_coefficients();
      m_got_floor_plane = true;

      // TODO refactor
      Cluster<Point> originClust(origin_);
      origin_proj_ = originClust.get_plane_projection(m_floor_plane);
   }
}

void Engine2::load_and_filter(ClusterCloud & cc) 
{
   load_raw(cc);
   filter_floor(cc);
}

void Engine2::add_cloud_to_viewer(PointCloud::Ptr cloud, std::string name, int viewport)
{
   viewer->removePointCloud(name);
   viewer->addPointCloud(cloud, name, viewport);
}

void Engine2::add_cloud_to_viewer(PointCloud::Ptr cloud, std::string name, int viewport, int r, int g, int b)
{
   PointCloudColorHandlerCustom<Point>    handler(cloud, r, g, b);
   viewer->removePointCloud(name);
   viewer->addPointCloud(cloud, handler, name, viewport);
}

void Engine2::add_cluster_to_viewer(PointCloud::Ptr clusters, std::string name, int viewport, int r, int g, int b)
{
   viewer->removePointCloud(name);
//    PointCloudColorHandlerCustom<Point>  handler(clusters, r, g, b);
   viewer->addPointCloud(clusters, name, viewport);
   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, name);

}

void Engine2::axis_view_setup(int viewport)
{

   viewer->removeShape("originLine", viewport);
   viewer->addLine(origin_, origin_proj_, "originLine", viewport);

   Point xDifference;
   addXYZ(origin_proj_, m_cal.x_vector, xDifference);
   viewer->removeShape("xRegAxis", viewport);
   viewer->addLine(origin_proj_, xDifference, 214, 0, 244, "xRegAxis", viewport);

   Point yDifference;
   addXYZ(origin_proj_, m_cal.y_vector, yDifference);
   viewer->removeShape("yRegAxis", viewport);
   viewer->addLine(origin_proj_, yDifference, 255, 165, 0, "yRegAxis", viewport);
}

/* Constructor */
Engine2::Engine2(pcl::visualization::PCLVisualizer::Ptr vis,
      Logger * logger) :
	QObject(0),
   cam_cloud_(new PointCloud()),
   dummy_mutex_(new Mutex()),
   m_camera(cam_cloud_, dummy_mutex_),
    #ifndef NO_ENGINE
   m_robot(new Robot("/dev/gantry")),
    #else
    m_robot(NULL),
    #endif
   m_logger(logger)
   
{

	surface = NULL;

#ifndef NO_ENGINE
	stringstream limits;
    limits << m_robot->currentLimits();
	m_logger->log(limits);
#endif


   position_valid = false;

   m_got_floor_plane = false;

   m_objects = new std::vector<WSObject>();

   m_state.state.x_axis_calibrated = false;

   // Origin
   origin_.x = 0; origin_.y = 0; origin_.z = 0;

   if (vis) viewer = vis;

   // Setup viewports
   /* 4 Viewports 
   viewer->createViewPort(0.0, 0.0, 0.5, 0.5, vp_calibration_clouds);
   viewer->createViewPort(0.5, 0.0, 1.0, 0.5, vp_calibration_axes);
   viewer->createViewPort(0.0, 0.5, 0.5, 1.0, vp_navigation);
   viewer->createViewPort(0.5, 0.5, 1.0, 1.0, vp_workspace); */

   viewer->createViewPort(0.0, 0.0, 1.0, 0.5, vp_calibration_axes);
   viewer->createViewPort(0.0, 0.5, 1.0, 1.0, vp_navigation);

	z_ceiling = 200.0;
	{
		std::stringstream msg;
		msg << "z_ceiling initially set to: " << z_ceiling;
		m_logger->log(msg);
	}

}

bool Engine2::add_objects(ClusterCloud & cloud)
{
// RobotPosition currentPos = getPosition();

   typedef     PointCloud::iterator  Iterator;

   Iterator closest_cluster;

   float       shortest_distance = -1.0;

   stringstream ss;
   ss << "Processing " << cloud.size() << " objects." << endl;
   m_logger->log(ss);

   // TODO reset the colors of all WSObjects, e.g. to default, before we overwrite them

   // Find the point with the shortest distance.
   for (Iterator i = cloud.begin(); i != cloud.end(); ++i)
   {
      Cluster<Point> current_point_cluster(*i);
      Point floorPoint = current_point_cluster.get_plane_projection(m_floor_plane);

      // Get the norm distance between floorPoint and originProjection
      float distance = PCLUtils::distance(floorPoint, origin_proj_);

      if (shortest_distance < 0 || distance < shortest_distance) {
         shortest_distance = distance;
         closest_cluster = i;
      } 
   } // for

   for (Iterator i = cloud.begin(); i != cloud.end(); ++i)
   {
      if (i == closest_cluster) {
         i->r = 0; i->g = 255; i->b = 255;
         add_object(*i, true);
      } else {
         i->r = 0; i->g = 255; i->b = 0;
         add_object(*i, false);
      }
   }

   add_cluster_to_viewer(cloud.clusters, "CurrentCloud_Clusters", vp_navigation);

   return true;
}

int
Engine2::move_to_object(int index)
{
   // Check if the index is valid
   if (index < 0 || index >= m_objects->size()) {
      std::stringstream emsg;
      emsg << "Engine2::move_to_object() - Illegal index (" << index << ") provided. m_objects->size() = "
         << m_objects->size() << ".";
      m_logger->log(emsg);
      return FAILURE;
   }

   std::vector<WSObject>::iterator object = m_objects->begin();
   std::advance(object, index);

   vantage_point(object);

   { // Message block
      std::stringstream msg;
      msg << "Picking up Object [" << object->id << "].";
      m_logger->log(msg);
   }

    create_closest_surface_map(current_view, object);


   pickup(object);

   // Re-load the point cloud

	load_raw(current_view);
	add_cloud_to_viewer(current_view.cloud, "CurrentCloud", vp_navigation);


   return SUCCESS;

}

void
Engine2::setup_claw_line(int viewport)
{
   // TODO should check that we have calibration before doing this
   viewer->removeShape("OriginLine", viewport);
   viewer->addLine(origin_, origin_proj_, "OriginLine", viewport);

   move_claw_line(m_cal.x_adj_amt, m_cal.y_adj_amt, 0);

}

void
Engine2::clear_objects()
{
   m_logger->log("Erasing all objects in workspace.");
   m_objects->clear();
}


Engine2::Point Engine2::get_claw_center_projection()
{
   Point claw_base, x_vec, y_vec;
   Point combined_vec;
   
   multiplyXYZ(m_cal.x_vector, m_cal.x_adj_amt, x_vec);
   multiplyXYZ(m_cal.y_vector, m_cal.y_adj_amt, y_vec);

   addXYZ(x_vec, y_vec, combined_vec);
   addXYZ(origin_, combined_vec, claw_base);
   
   Cluster<Point> claw_center_clust(claw_base);

   // TODO make the floor plane a member of the calibration class.
   Point claw_center_proj = claw_center_clust.get_plane_projection(m_floor_plane);

   return claw_center_proj;
}


void 
Engine2::move_claw_line(float x_amount, float y_amount, int viewport)
{
   Point claw_base;
   Point x_vec, y_vec;
   Point combined_vec;

   m_cal.x_adj_amt = x_amount;
   m_cal.y_adj_amt = y_amount;

   multiplyXYZ(m_cal.x_vector, x_amount, x_vec);
   multiplyXYZ(m_cal.y_vector, y_amount, y_vec);

   addXYZ(x_vec, y_vec, combined_vec);
   addXYZ(origin_, combined_vec, claw_base);

   
   Cluster<Point> claw_center_clust(claw_base);
   Point claw_center_proj = claw_center_clust.get_plane_projection(m_floor_plane);

   ClusterCloud claw_proj_cloud;
   claw_proj_cloud.cloud->push_back(claw_base);
   claw_proj_cloud.cloud->push_back(claw_center_proj);
   add_cluster_to_viewer(claw_proj_cloud.cloud, "claw_proj_cloud", viewport, 255, 0, 0);

   viewer->removeShape("ClawLine");
   viewer->addLine(claw_base, claw_center_proj, 255, 0, 0, "ClawLine", viewport);
}
