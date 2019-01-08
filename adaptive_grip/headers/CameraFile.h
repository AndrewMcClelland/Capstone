#ifndef CAMERAFILE__H
#define CAMERAFILE__H

/*
   Name        :  CameraFile.h
   Author      :  Aaron
   Purpose     :  An extension of CameraBase used to retrive point clouds from
                  a file, instead of a Kinect. 
                  The typename PointT determines what `kind` of points should be 
                  retrieved from file (can be pcl::PointXYZ, 
                  pcl::PointXYZRGB, pcl::PointXYZRGBA, etc..)

                  Note that this is a templated class so functions have to be 
                  implemented where they are declared.
*/

#include <iostream>
#include "Camera.h"
#include <pcl/io/pcd_io.h>
#include <string>

template <typename PointT> class CameraFile: public CameraBase<PointT> {

   typedef typename pcl::PointCloud<PointT>              Cloud;
   typedef typename pcl::PointCloud<PointT>::Ptr         CloudPtr;
   typedef typename pcl::PointCloud<PointT>::ConstPtr   CloudConstPtr;

   public:

   // Constructor.
   // Takes in a Mutex along with a point cloud, to make sure the cloud isn't 
   // being modified or accessed elsewhere while it's being overwritten.
   CameraFile(CloudPtr cloud, Mutex * mutex);

   /*
      Function       : retrieve()
      Description    : Updates m_cloud from file.
      Returns        : Returns SUCCCESS if read from file was successful,
                       FAILURE otherwise.
   */
   int retrieve();

   void setFilename(std::string);
   

   private:

   // Pointer to cloud mutex
   Mutex * _cloud_mutex;
   std::string filename;
};
#endif
