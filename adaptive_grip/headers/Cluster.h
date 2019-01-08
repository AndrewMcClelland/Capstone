#ifndef CLUSTER__H
#define CLUSTER__H

// Class to represent a cluster in 3-D space.
// Implements some useful functions (projection on to a plane,
// distance to a plane)

template <typename PointT>
class Cluster {

public:
   // Location of cluster in 3-D space.
   PointT      m_location;

   // TODO get rid of this default constructor
   Cluster() {
      m_location.x = 999;
      m_location.y = 999;
      m_location.z = 999;
   }
   // Constructor
   Cluster(PointT _location) :
      m_location(_location) {}

   // Copy constructor
   Cluster(const Cluster & cluster) :
      m_location(cluster.m_location) {}

   Cluster(float _x, float _y, float _z) {
      m_location.x = _x;
      m_location.y = _y;
      m_location.z = _z;
   }


   // Project a point over a plane
   PointT      
   get_plane_projection(const Eigen::Vector4f & plane)
   {
      Eigen::Vector4f p (m_location.x, m_location.y, m_location.z, 1.0);

      float distance_to_plane = get_distance_to_plane(plane);

      Eigen::Vector4f projection_vector = p - plane * distance_to_plane;

      PointT projection = m_location; // Copy all fields besides x, y, z

      projection.x = projection_vector[0];
      projection.y = projection_vector[1];
      projection.z = projection_vector[2];
      
      return projection;

   }

   // Get the distance to a plane.
   float       get_distance_to_plane(const Eigen::Vector4f & plane) // CAUTION: The plane needs to be normalized!
   {
      Eigen::Vector4f p (m_location.x, m_location.y, m_location.z, 1.0); 
      return plane.dot(p);
   }


};

#endif
