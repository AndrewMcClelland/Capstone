#ifndef CAMERABASE__H
#define CAMERABASE__H

/*
   Name        :  CameraBase.h
   Author      :  Aaron
   Purpose     :  A virtual class used to retrieve point clouds from a Camera.
                  the typename PointT determines what `kind` of points the 
                  point cloud should represent (can be pcl::PointXYZ, 
                  pcl::PointXYZRGB, pcl::PointXYZRGBA, etc..)

                  Note that this is a templated class so functions have to be 
                  implemented where they are declared.
*/
template <typename PointT> class CameraBase {

   // Convenience typedefs
   typedef typename pcl::PointCloud<PointT>              Cloud;
   typedef typename pcl::PointCloud<PointT>::Ptr         CloudPtr;
   typedef typename pcl::PointCloud<PointT>::ConstPtr    CloudConstPtr;

   public: 

   // A pointer to a cloud object.
   CloudPtr       m_cloud;

   // Constructor.
   CameraBase(CloudPtr cloud) :
      m_cloud(cloud)
   {}

/*
   Function       : retrieve()
   Description    : Pure virtual function to update m_cloud from a camera.
   Returns        : An integer (SUCCESS, FAILURE) that depends on the function's success.
*/
   virtual int retrieve() = 0;

};

#endif
