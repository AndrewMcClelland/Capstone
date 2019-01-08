// AN - November 22 - this is working!
// This code tests if your machine can connect to a Kinect device
// and receive depth data/construct point clouds in real-time.
// If successfully ran this will open a window that shows point clouds
// being constructed from the Kinect in real time.

#include <boost/thread/thread.hpp>
 
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
 
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
 
/** pthread_mutex_t cpp wrapper */
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
public:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    Mutex * mutex;
 
    SimpleOpenNIViewer():
        viewer(new pcl::visualization::PCLVisualizer ("3D Viewer")),
        mutex(new Mutex)
    {}
 
    /** OpenNI callback. Each time a data pack arrives, the Mutex is locked,
    *   and the cloud is updated.
    */
    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
        mutex->lock();
        viewer->updatePointCloud(cloud, "kinect_cloud");
        mutex->unlock();
    }
 
    void run ()
    {
        // Start off by creating an empty point cloud (with support for depth (XYZ) and color (RGBA)))
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr kinect_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
        // Add the empt cloud to the viewer. The cloud will be updated with every openni callback
        viewer->addPointCloud<pcl::PointXYZRGBA> (kinect_cloud_ptr, "kinect_cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "kinect_cloud");
        // Create the openni interface, and set cloud_cb_ as the callback
     // pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();
        pcl::Grabber* interface = new pcl::OpenNIGrabber();

        // Had to change the function template from PointXYZ to PointXYZ
        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
                boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
        // Start the interface
        interface->registerCallback (f);
        interface->start ();
 
        while (!viewer->wasStopped())
        {   
            // Aquire the mutex, to make sure that the cloud isn't being updated
            mutex->lock();
            // Redraw cloud
            viewer->spinOnce (20);
            // Release mutex
            mutex->unlock();
        }
        interface->stop ();
    }
};
 
int main ()
{
    char key[1];
//  cout << "Press Enter..." << endl;
//  fgets(key, 1, stdin);
//  cout << "Continuing" << endl;
    SimpleOpenNIViewer v;
    v.run ();
    return 0;
}
