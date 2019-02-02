#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include  <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <cmath>

using namespace std;
using namespace cv;

class CircleParams
{
  public:
  Point2f center;
  float radius;

  CircleParams(Point2f newCenter = Point2f(-1, -1), float newRadius = -1) {
    center = newCenter;
    radius = newRadius;
  }

  friend ostream& operator<<(ostream& os, const CircleParams &cp) {
      os << "Centered at: [" << cp.center.x << ", " << cp.center.y << "]\tradius = " << cp.radius;
      return os;
  }
};


int main(int argc, char **argv) {

  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;

  bool kinect_shutdown = false;

  if(freenect2.enumerateDevices() == 0)
  {
    cout << "no device connected!" << endl;
    return -1;
  }

  string serial = freenect2.getDefaultDeviceSerialNumber();

  pipeline = new libfreenect2::OpenCLPacketPipeline();


  dev = freenect2.openDevice(serial, pipeline);

  if (dev == 0)
  {
	cerr << "No device opened!"<<endl;
	return -1;
  }

  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;

  dev->setIrAndDepthFrameListener(&listener);

  dev->start();

  cout << "device serial: " << dev->getSerialNumber() << endl;
  cout << "device firmware: " << dev->getFirmwareVersion() << endl;

  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);

  Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;



   while(!kinect_shutdown)
  {
    listener.waitForNewFrame(frames);

    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);

    // Convert the depth matrix to range [0,1] instead of [0,4096]
    depthmat = depthmat / 4096.0f;

    // Variables for threholding and contour
    Mat thresh,thresh2, contour;
    vector<vector<Point> > contours;
    vector<CircleParams> circles;
    CircleParams max_circle;
    max_circle.radius = 0;
    // Threshold the depth so only hand will be visible in certain depth window
    threshold( depthmat, thresh, 0.16f, 1, 3);
    threshold( thresh, thresh2, 0.20f, 1, 4);

    // Convert 32 bit depth matrix into 8 bit matrix for contour identification
    // also function multiplies the matrix by 255 increasing the range [0, 255]
    thresh2.convertTo(contour,CV_8UC1, 255 );

    // helper function finds countur in 8 bit depth matrix and returns
    // a hierarchical vector of contours (vector<vector<Point>>)
    findContours(contour, contours, 0, 1);

    // Iterating through the vector containing Point vectors
    // to find the center of the contoured circle
    float max_radius = 0;
    Point2f max_center;
    for (vector<vector<Point>>::iterator it=contours.begin(); it < contours.end(); it++)
    {
      Point2f center;
      float radius;
      vector<Point> pts = *it;
      // converting contour points into CV matrix
      Mat pointsMat = Mat(pts);
      // Finding a bounding circle from hand contours
      minEnclosingCircle(pointsMat, center, radius);
      //containing each enclosed contour group
      CircleParams params = CircleParams(center, radius);
      circles.push_back(params);

      // Setting params for max circle contour (the hand)
      if(radius > max_radius) {
        max_radius = radius;
        max_center = center;
      }
    }

    // Setting params for max circle contour (the hand)
    max_circle.center = max_center;
    max_circle.radius = max_radius;
    // cout << "Centered at: [" << max_circle.center.x << ", " << max_circle.center.y << "]\tradius = " << max_circle.radius << endl;
    cout << max_circle << endl;

    // !!! TODO: Add gesture recognition here of the circle by checking the difference
    // in radius of the circle with the previous frame

    // Add a circle drawing to the depth matrix if hand is detected
    if (max_circle.radius)
    {
      Scalar color(255, 255, 255 );
      circle(contour, max_circle.center, max_circle.radius, color);
    }

    imshow("Depth Matrix", contour );

    int key = cv::waitKey(1);

    listener.release(frames);
  }

  dev->stop();
  dev->close();


  delete registration;
  return  0;
}
