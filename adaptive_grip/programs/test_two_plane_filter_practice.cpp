#include "Shell.h"
#include "TwoPlaneFilter.h"
#include "CameraFile.h"
#include "common_defs.h"

#include <iostream>

using stringmanip::arg_list;
using std::cout;
using std::endl;

// Forward declarations (what does he mean: by this?)
typedef pcl::PointXYZRGB    Point

/*
	TwoPlaneFilterShell is an extension of Shell. This means that it inherits the funct	   ions and properties of Shell, and overloads their functions as necessary.
*/
class TwoPlaneFilterShell : public Shell {

	public:

		typedef pcl::PointXYZRGBA Point;
		
		// Mutex prevents others from calling the same thing at the same time
		// cloud : a point cloud?
		// camera: a point cloud file based on what the camera sees?
		Mutex *				mutex;
		pcl::PointCloud<Point>::Ptr	cloud;
		CameraFile<Point>		camera;

/*
	Function	: reload_cloud
	Arguments	: arg_list (ignored)
	Description	: Clears and re-loads the Point Cloud 'cloud' using the 'camera
			  object.
*/	  
	void reload_cloud(arg_list args)
	{
		//Clears the point cloud 'cloud'
		cloud->clear();
		
		//Retrieves a new point cloud from the camera
		camera.retrieve();

		//Sends message and updates cloud to be equivalent to camera?
		cout << "Reloaded cloud." << endl;
		viewer->updatePointCloud<Point>(cloud);
	}

/*
	Function	: filter
	Arguments	: arg_list
			  Should contain one string representing a floating point number.
	Description	: ?
*/
	void filter(arg_list args)
	{
		// What does this do with respect to the code? Understood that it gets
		// how much to separate the two planes
		double separation_amt = atof(args.front().c_str());
		cout << "Separation amount: " << separation_amt << endl;

		/* Parse Separation Amount */
		TwoPlaneFilter<Point> filt(cloud, mutex, separation_amt);
		filt.filter_plane();

		viewer->updatePointCloud<Point>(cloud);
	}

/*
	Function	: say_hello
	Arguments	: arg_list
	Description	: Tells the computer to say 'hello' back upon being commanded
*/
	void say_hello()
	{
		cout << "Hello, motherfucker." << endl;
	}
/*
	Function	: bind_functions()
	Description	: Registers the functions above with the shell.
*/
	void bind_functions()
	{
      		Shell::callback reload = boost::bind(&TwoPlaneFilterShell::reload_cloud, this, _1);
      		Shell::callback filter = boost::bind(&TwoPlaneFilterShell::filter, this, _1);
		Shell::callback said_hello = boost::bind(&TwoPlaneFilterShell::said_hello, this, _1);
      		register_function(reload, "rel", "Re-load the point cloud from file.");
      		register_function(filter, "fil", "Filter the point cloud. [fil %d]");
		register_function(said_hello, "say", "Say hello.");
	}
	
	// Constructor
	TwoPlaneFilterShell() :
		Shell(),
		cloud(new pcl::PointCloud<Point>()),
		mutex(new Mutex()),
		camera(cloud, mutex)
	{
		viewer->addPointCloud<Point>(cloud, "cloud");
		bind_functions();
	}
};

int main (int argc, char * argv[])
{
	TwoPlaneFilterShell shell;
	shell.run_shell();

	return SUCCESS;
}
