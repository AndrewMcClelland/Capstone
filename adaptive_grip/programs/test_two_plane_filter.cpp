#include "Shell.h"
#include "Mutex.h"
#include "TwoPlaneFilter.h"
#include "CameraFile.h"
#include "common_defs.h"

#include <iostream>

using stringmanip::arg_list;
using std::cout;
using std::endl;

// Forward declarations
typedef pcl::PointXYZRGB   Point;


class TwoPlaneFilterShell : public Shell {
   
public:

   typedef pcl::PointXYZRGBA Point;

   // Note to self; members initialized using in an initializer list
   // are initialized in the order they are declared in the class definiton.
   // That's why "mutex" and "cloud" need to be declared before "camera".
   // Make sure to use the -Wuninitialized (or -Wall) warning flag when compiling.
   Mutex *                          mutex;
   pcl::PointCloud<Point>::Ptr      cloud;
   CameraFile<Point>                camera; 


/*
   Function       : reload_cloud
   Arguments      : arg_list (ignored)
   Decription     : Clears and re-loads the Point Cloud `cloud`
                    using the `camera` object.
*/
   void reload_cloud(arg_list args)
   {
      cloud->clear();
      camera.retrieve();
      cout << "Reloaded cloud." << endl;
      viewer->updatePointCloud<Point>(cloud);
   }

/*
   Function       : filter
   Arguments      : arg_list 
                    Should contain one string representing a floating point number.
   Decription     : 
                    
*/
   void filter(arg_list args)
   {
      
      double separation_amt = atof(args.front().c_str());
      cout << "Separation amount: " << separation_amt << endl;

      /* Parse Separation Amount */
      TwoPlaneFilter<Point>  filt(cloud, mutex, separation_amt);
      filt.filter_plane();

      viewer->updatePointCloud<Point>(cloud);
   }

/*
   Function       : bind_functions()
   Decription     : Registers the 'filter' and 'reload_cloud' functions with the shell.
                    
*/
   void bind_functions()
   {
      Shell::callback reload = boost::bind(&TwoPlaneFilterShell::reload_cloud, this, _1);
      Shell::callback filter = boost::bind(&TwoPlaneFilterShell::filter, this, _1);
      register_function(reload, "rel", "Re-load the point cloud from file.");
      register_function(filter, "fil", "Filter the point cloud. [fil %d]");
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

// MAIN Method: Create the shell and let it run.
int main (int argc, char * argv[])
{
   TwoPlaneFilterShell shell;
   shell.run_shell();

   return SUCCESS;
}
