// boost multithreading
#include <boost/thread/thread.hpp>
//
// common headers
#include <pcl/common/common_headers.h>
// PCL I/O : Grabber interface
#include <pcl/io/pcd_io.h>

// PCL I/O : OpenNIGrabber interface
#include <pcl/io/openni_grabber.h>

// PCL Visualization: Visualizer interface
#include <pcl/visualization/pcl_visualizer.h>

// Debug log macro
#define LOG(x) std::cout << x << std::endl

using std::cout;
using std::endl;
 
/** pthread_mutex_t cpp wrapper */
// Not too sure how this works. 
// For now: just remember to lock/un-lock the cloud_mutex whenever you are changing
// a PointCloudT object that can be changed from multiple places concurrently
class Mutex
{
public:
    Mutex() { pthread_mutex_init(&m_mutex, NULL); }
    void lock() { pthread_mutex_lock(&m_mutex); }
    void unlock() { pthread_mutex_unlock(&m_mutex); }
 
    class ScopedLock
    {
    public:
        ScopedLock(Mutex &mutex) : _mutex(mutex) { _mutex.lock(); }
        ~ScopedLock() { _mutex.unlock(); }
    private:
        Mutex &_mutex;
    };
private:
    pthread_mutex_t m_mutex;
};
 
/** class to handle the drawing and openni callbacks */
class SimpleOpenNIViewer
{

   // Point Cloud typedef
   typedef pcl::PointCloud<pcl::PointXYZRGBA>   cloud_t;

public:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    // Mutex for updating the Point Cloud
    Mutex * m_cloud_mutex;

    // Mutex for handling keystrokes
    Mutex * m_ks_mutex;
 
    SimpleOpenNIViewer():
        viewer(new pcl::visualization::PCLVisualizer ("3D Viewer")),
        m_cloud_mutex(new Mutex),
        m_ks_mutex(new Mutex),
        m_save_cloud(false),
        m_update_cloud(true)
    {}

    // Set to true when we have a key event to handle
    bool       m_save_cloud;
    bool       m_update_cloud;
    cloud_t    m_cloud;

 
    /** OpenNI callback. Each time a data pack arrives, the Mutex is locked,
    *   and the cloud is updated.
    */
    void cloud_cb_ (const cloud_t::ConstPtr &cloud)
    {
        if (m_update_cloud == true) {
           m_cloud_mutex->lock();

           viewer->updatePointCloud(cloud, "kinect_cloud");

           m_cloud_mutex->unlock();
        }

        if (m_save_cloud == true)
        {

           // Lock the mutex so the cloud can't be updated while we're copying
           m_cloud_mutex->lock();

           // Copy the point cloud from the kinect into our internal
           // point cloud ("m_cloud").
           m_cloud = (*cloud);
           m_cloud_mutex->unlock();

           // Write the saved cloud to a file.
           pcl::io::savePCDFileASCII("saved_kinect_pcd.pcd", m_cloud);
           std::cout << "Saved " << m_cloud.points.size() << " data points to 'saved_kinect_pcd.pcd" << std::endl;
           
           // Un-set the 'save cloud' flag.
           m_save_cloud = false;
        }

    }


    // Key-Pressed callback
    void key_pressed_cb_ (const pcl::visualization::KeyboardEvent& ke)
    {
       if (ke.keyDown()) {
         m_ks_mutex->lock();
         if (ke.getKeyCode() == 'C' || ke.getKeyCode() == 'c')
         {
            // Set 'm_save_cloud' variable
            m_save_cloud = true;
            
            LOG("Saving cloud.");
         }

         if (ke.getKeyCode() == 'B' || ke.getKeyCode() == 'b')
         {
            m_update_cloud = !m_update_cloud;
            LOG("Setting updates to: " << m_update_cloud);
         }
         m_ks_mutex->unlock();
       }
    }
   
 
    void run ()
    {
        // Start off by creating an empty point cloud (with support for depth (XYZ) and color (RGBA)))
        cloud_t::Ptr kinect_cloud_ptr (new cloud_t);
        // Add the empt cloud to the viewer. The cloud will be updated with every openni callback
        viewer->addPointCloud<pcl::PointXYZRGBA> (kinect_cloud_ptr, "kinect_cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "kinect_cloud");
        // Create the openni interface, and set cloud_cb_ as the callback
     // pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();
        pcl::Grabber* interface = new pcl::OpenNIGrabber();

        // Had to change the function template from PointXYZRGBA to PointXYZRGBA
        boost::function<void (const cloud_t::ConstPtr&)> callback_function =
                boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
        interface->registerCallback (callback_function);

        // NEW: Also create a boost function template for our m_save_cloud callback
        // boost::bind is confusing here.
        //    First argument: a non-static function
        //    Second argument: an instance of the class, which is technically the first
        //     argument to any non-static function
        //    Third argument: the 'first' argument to the function, which is technically 
        //     the second argument to any non-static function
        boost::function<void (const pcl::visualization::KeyboardEvent&)> key_pressed_f = 
           boost::bind( &SimpleOpenNIViewer::key_pressed_cb_, this, _1);

        viewer->registerKeyboardCallback(key_pressed_f);

        // Start the interface
        interface->start ();
 
        while (!viewer->wasStopped())
        {   
            // Aquire the m_cloud_mutex, to make sure that the cloud isn't being updated
            m_cloud_mutex->lock();
            // Redraw cloud - 
            viewer->spinOnce (50, false);
            // Release m_cloud_mutex
            m_cloud_mutex->unlock();
        }
        interface->stop ();
    }
};


// Let's try to capture a .pcd file from the kinect and write it to file.
int main ()
{
   SimpleOpenNIViewer v;
   v.run();
   return 0;
}
