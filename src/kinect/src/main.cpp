#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include  <opencv2/opencv.hpp>

// #include "../include/kinect_to_gantry.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>  // for inner_product

// Kinect Serial Numbers
#define KINECT_ARM_MOUNT_SERIAL 505083742542
#define KINECT_HAND_RECOG_SERIAL 502327242542

// Finger recognition stuff
#define NUM_FRAMES_FINGER_AVG 100
#define NUM_FINGERS_STD_DEV 0.5

// Movement for the +/- x and y coordinates when doing the non hand-positioned based commands
#define FIXED_GANTRY_MOVEMENT 300

// Taken from robot_limits.h
#define GANTRY_MIN_X 0
#define GANTRY_MAX_X 3000
#define GANTRY_MIN_Y -1795
#define GANTRY_MAX_Y 0

// Gantry coordinates/inch 
#define GANTRY_X_COORD_PER_INCH 3000.0f / 132.0f
#define GANTRY_Y_COORD_PER_INCH 1750.0f / 83.0f

// Kinect inch range
#define ARM_KINECT_X_RANGE_INCHES 58.0f
#define ARM_KINECT_Y_RANGE_INCHES 72.0f

#define ARM_KINECT_X_RANGE_COORDS ARM_KINECT_X_RANGE_INCHES * GANTRY_X_COORD_PER_INCH
#define ARM_KINECT_Y_RANGE_COORDS ARM_KINECT_Y_RANGE_INCHES * GANTRY_Y_COORD_PER_INCH

// #define CONV_X_KINECT_TO_GANTRY (GANTRY_MAX_X - GANTRY_MIN_X) / (KINECT_MAX_X - KINECT_MIN_X)
// #define CONV_Y_KINECT_TO_GANTRY (GANTRY_MAX_Y - GANTRY_MIN_Y) / (KINECT_MAX_Y - KINECT_MIN_Y)


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
      os << "Centered at (normal (x,y notation)): [" << cp.center.y << ", " << cp.center.x << "]\tradius = " << cp.radius;
      return os;
  }
};

float innerAngle(float px1, float py1, float px2, float py2, float cx1, float cy1)
{

  float dist1 = std::sqrt(  (px1-cx1)*(px1-cx1) + (py1-cy1)*(py1-cy1) );
  float dist2 = std::sqrt(  (px2-cx1)*(px2-cx1) + (py2-cy1)*(py2-cy1) );

  float Ax, Ay;
  float Bx, By;
  float Cx, Cy;

  //find closest point to C  
  //printf("dist = %lf %lf\n", dist1, dist2);  

  Cx = cx1;
  Cy = cy1;
  if(dist1 < dist2)
  {
    Bx = px1;
    By = py1;
    Ax = px2;
    Ay = py2;


  }else{
    Bx = px2;
    By = py2;
    Ax = px1;
    Ay = py1;
  }


  float Q1 = Cx - Ax;
  float Q2 = Cy - Ay;
  float P1 = Bx - Ax;
  float P2 = By - Ay;


  float A = std::acos( (P1*Q1 + P2*Q2) / ( std::sqrt(P1*P1+P2*P2) * std::sqrt(Q1*Q1+Q2*Q2) ) );

  A = A*180/CV_PI;

  return A;
}

bool fingerToGesture(const int numFingers, const Point2f& center, const int kinectNumRows, const int kinectNumColumns) {
  switch (numFingers) {

    // Move gantry arm based on the gantry coordinate system
    case 0: {

      // Conversion factor for hand recognition to gantry coordinates
      float kinect_to_gantry_x = (float)(GANTRY_MAX_X - GANTRY_MIN_X) / kinectNumColumns;
      float kinect_to_gantry_y = (float)(GANTRY_MAX_Y - GANTRY_MIN_Y) / kinectNumRows;

        // Coordinates of hand with respect to the gantry coordinates
      int x_gantry = int(center.x * kinect_to_gantry_x) + GANTRY_MIN_X;
      int y_gantry = int(center.y * kinect_to_gantry_y) + GANTRY_MIN_Y;

      // Limit x and y coordiantes to be within their min and max values (inclusive)
      x_gantry = x_gantry < GANTRY_MIN_X ? GANTRY_MIN_X : x_gantry;
      x_gantry = x_gantry > GANTRY_MAX_X ? GANTRY_MAX_X : x_gantry;
      y_gantry = y_gantry < GANTRY_MIN_Y ? GANTRY_MIN_Y : y_gantry;
      y_gantry = y_gantry > GANTRY_MAX_Y ? GANTRY_MAX_Y : y_gantry;

      cout << "movetoloop " + to_string(x_gantry) + " " + to_string(y_gantry) + " 0" << endl;
    }
    break;

    // Move gantry arm based on the mounted kinect v1 coordinate system
    case 1: {
      // Conversion factor for hand recognition to gantry coordinates
      float kinect_to_mountedKinect_x = (float)ARM_KINECT_X_RANGE_COORDS / kinectNumColumns;
      float kinect_to_mountedKinect_y = (float)ARM_KINECT_Y_RANGE_COORDS/ kinectNumRows;
      
      // Coordinates of hand with respect to the arm-mounted Kinect coordinates
      int x_kinect = int(center.x * kinect_to_mountedKinect_x);
      int y_kinect = int(center.y * kinect_to_mountedKinect_y);

      // Adjust coordinates so centre of the kinect's view is the origin
      x_kinect -= int(kinectNumColumns * kinect_to_mountedKinect_x) / 2;
      y_kinect -= int(kinectNumRows * kinect_to_mountedKinect_y) / 2;

      cout << "moveby " + to_string(x_kinect) + " " + to_string(y_kinect) + " 0" << endl;
    }
      break;

    // Move gantry arm in the 'x' direction by distance FIXED_GANTRY_MOVEMENT 
    case 2: {
      // Move gantry in negative x direction if hand is on left half of kinect view
      if(center.x <= kinectNumColumns / 2) {
        cout << "moveby " + to_string(FIXED_GANTRY_MOVEMENT * -1) + " 0 0" << endl;
      }
      // Move gantry in positive x direction if hand is on right half of kinect view
      else {
        cout << "moveby " + to_string(FIXED_GANTRY_MOVEMENT) + " 0 0" << endl;
      }
    }
      break;

    // Move gantry arm in the 'y' direction by distance FIXED_GANTRY_MOVEMENT 
    case 3: {
      // Move gantry in negative y direction if hand is on bottom half of kinect view
      if(center.y >= kinectNumRows / 2) {
        cout << "moveby 0 " + to_string(FIXED_GANTRY_MOVEMENT * -1) + " 0" << endl;
      }
      // Move gantry in positive y direction if hand is on top half of kinect view
      else {
        cout << "moveby 0 " + to_string(FIXED_GANTRY_MOVEMENT) + " 0" << endl;
      }
    }
      break;

    case 4:
      cout << "Executing num fingers case 4" << endl;
      break;

    case 5:
      cout << "Executing num fingers case 5" << endl;
      break;

    default:
      cout << "Error!!! Num fingers = " << numFingers << endl;
      return false;
      break;
  }
  return true;
}

int main(int argc, char **argv) {

  libfreenect2::Freenect2 freenect2_HAND;
  // libfreenect2::Freenect2 freenect2_GRIP;
  libfreenect2::Freenect2Device *dev_HAND = 0;
  // libfreenect2::Freenect2Device *dev_GRIP = 0;
  libfreenect2::PacketPipeline *pipeline_HAND = 0;
  // libfreenect2::PacketPipeline *pipeline_GRIP = 0;

  // Suppressing output from the libfreenect2 logger (comment it out to make it log info to output)
  libfreenect2::setGlobalLogger(NULL);

  bool kinect_shutdown = false;

  if(freenect2_HAND.enumerateDevices() == 0)
  {
    cout << "no device connected!" << endl;
    return -1;
  }

  // if(freenect2_GRIP.enumerateDevices() == 0)
  // {
  //   cout << "no device connected!" << endl;
  //   return -1;
  // }

  // string serial = freenect2.getDefaultDeviceSerialNumber();

  // pipeline_GRIP = new libfreenect2::OpenCLPacketPipeline();
  pipeline_HAND = new libfreenect2::OpenCLPacketPipeline();

  // dev_GRIP = freenect2_GRIP.openDevice(to_string(KINECT_ARM_MOUNT_SERIAL), pipeline_GRIP);
  dev_HAND = freenect2_HAND.openDevice(to_string(KINECT_HAND_RECOG_SERIAL), pipeline_HAND);

  if (dev_HAND == 0)
  {
	cerr << "Hand kinect not opened!"<<endl;
	return -1;
  }

  //  if (dev_GRIP == 0)
  // {
	// cerr << "Grip kinect not opened!"<<endl;
	// return -1;
  // }

  libfreenect2::SyncMultiFrameListener listener_HAND(libfreenect2::Frame::Color | libfreenect2::Frame::Depth );
  // libfreenect2::SyncMultiFrameListener listener_GRIP(libfreenect2::Frame::Color | libfreenect2::Frame::Depth );

  libfreenect2::FrameMap frames_HAND;
  // libfreenect2::FrameMap frames_GRIP;

  dev_HAND->setIrAndDepthFrameListener(&listener_HAND);
  dev_HAND->setColorFrameListener(&listener_HAND);

  dev_HAND->start();

  // dev_GRIP->setIrAndDepthFrameListener(&listener_GRIP);
  // dev_GRIP->setColorFrameListener(&listener_GRIP);

  // dev_GRIP->start();

  libfreenect2::Registration* registration_HAND = new libfreenect2::Registration(dev_HAND->getIrCameraParams(), dev_HAND->getColorCameraParams());
  // libfreenect2::Registration* registration_GRIP = new libfreenect2::Registration(dev_GRIP->getIrCameraParams(), dev_GRIP->getColorCameraParams());
  
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);

  Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2, hsv, combinedRGBDepthMat, depthMatConverted;

  int curr_num_finger_frames = 0, total_count_fingers = 0;
  float avg_num_fingers = -1, sq_sum;
  vector<double> fingers_in_frame;
  double finger_stdev;

  // OpenCV resize parameters
  int screen_res_width = 1280, screen_res_height = 1024;
  int scale_width, scale_height, scale;
  int resized_window_width, resized_window_height;

  // Values used to alter the HSV bounds of the resulting image
  // DONT THINK THE HSV CURRENTLY WORKS BECAUSE THE INPUT FRAME IS DEPTH DATA ONLY...HAVE TO CONFIRM WITH LIPSKI
  int minH = 122, maxH = 180, minS = 27, maxS = 74, minV = 118, maxV = 198, depthThresholdMin = 0.16f, depthThresholdMax = 0.20f;
  const char* windowName = "Fingertip detection";
  cv::namedWindow(windowName);
  cv::createTrackbar("MinH", windowName, &minH, 180);
  cv::createTrackbar("MaxH", windowName, &maxH, 180);
  cv::createTrackbar("MinS", windowName, &minS, 255);
  cv::createTrackbar("MaxS", windowName, &maxS, 255);
  cv::createTrackbar("MinV", windowName, &minV, 255);
  cv::createTrackbar("MaxV", windowName, &maxV, 255);
  cv::createTrackbar("minDepthThreshold", windowName, &depthThresholdMin, 60);
  cv::createTrackbar("depthThresholdMax", windowName, &depthThresholdMax, 60);

  // // Testing converting depth to colour frame
  // while(!kinect_shutdown) { 
  //   libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
  //   libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
  //   libfreenect2::Frame bigDepth(1920,1082,4);

  //   registration->apply(rgb, depth, &undistorted, &registered,&bigDepth);
  //   cv::Mat bigDepthMat;
  //   cv::Mat(bigDepth.height, bigDepth.width, CV_32FC1, bigDepth.data).copyTo(bigDepthMat);
  //   // cv::imshow("registeredInv", bigDepthMat / 4096.0f);
    


  //   int key = cv::waitKey(1);
  //   listener.release(frames);
  // }

  while(!kinect_shutdown) {
    listener_HAND.waitForNewFrame(frames_HAND);
    // listener_GRIP.waitForNewFrame(frames_GRIP);

    libfreenect2::Frame *depth = frames_HAND[libfreenect2::Frame::Depth];
    libfreenect2::Frame *rgb = frames_HAND[libfreenect2::Frame::Color];
    
    // libfreenect2::Frame bigDepth(1920s,1082,4);

    // Variables for threholding and contour
    Mat thresh,thresh2, contour;
    vector<vector<Point> > contours;
    vector<CircleParams> circles;
    CircleParams max_circle;
    max_circle.radius = 0;

    // Get individual RGB & Depth Frames
    Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
    Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);

    // Convert the depth matrix to range [0,1] instead of [0,4096]
    depthmat = depthmat / 4096.0f;

    // Threshold the depth so only hand will be visible in certain depth window
    threshold( depthmat, thresh, depthThresholdMin / 100.0f, 1, 3);
    threshold( thresh, thresh2, depthThresholdMax / 100.0f, 1, 4);

    // Convert RGB vals to HSV and limit frames to be within HSV bounds
    cv::cvtColor(rgbmat, hsv, COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV), hsv);

    // // Create combined RGB/Depth Frame (have to convert depth matrix to correct CV_8U type first)
    // depthmat.convertTo(depthMatConverted, CV_8UC4);
    // Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(combinedRGBDepthMat, depthMatConverted);
    // combinedRGBDepthMat.copyTo(combinedRGBDepthMat, rgbmat);

    // Convert 32 bit depth matrix into 8 bit matrix for contour identification
    // also function multiplies the matrix by 255 increasing the range [0, 255]
    thresh2.convertTo(contour,CV_8UC1, 255 );
    // hsv.convertTo(contour,CV_8UC1, 255 );
    // combinedRGBDepthMat.convertTo(contour,CV_8UC1, 255 );

    // helper function finds countur in 8 bit depth matrix and returns
    // a hierarchical vector of contours (vector<vector<Point>>)
    
    // findContours(contour, contours, 0, 1);
    std::vector<cv::Vec4i> hierarchy;
    findContours(contour, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // Iterating through the vector containing Point vectors
    // to find the center of the contoured circle

    size_t largest_contour = 0;
    Point2f max_center;
    float max_radius = -1;
    for (size_t i = 1; i < contours.size(); i++)
    {
      Point2f center;
      float radius;
      vector<Point> pts = contours.at(i);
      // converting contour points into CV matrix
      Mat pointsMat = Mat(pts);
      // Finding a bounding circle from hand contours
      minEnclosingCircle(pointsMat, center, radius);
      //containing each enclosed contour group
      CircleParams params = CircleParams(center, radius);
      circles.push_back(params);

      // Setting x,y and radius for max circle contour (the hand)
      if(radius > max_radius) {
        max_radius = radius;
        max_center = center;
      }

      // Getting actual contour of the hand
      if (cv::contourArea(contours[i]) > cv::contourArea(contours[largest_contour])) {
        largest_contour = i;
      }
    }

    // Setting params for max circle contour (the hand)
    max_circle.center = max_center;
    max_circle.radius = max_radius;

    // Draw contour outline of hand
    cv::drawContours(contour, contours, largest_contour, cv::Scalar(255, 0, 0), 1);

    // Add a circle drawing to the depth matrix if hand is detected
    // circle(contour, max_circle.center, max_circle.radius, cv::Scalar(255, 255, 255));

    if (!contours.empty())
      {
          std::vector<std::vector<cv::Point> > hull(1);
          cv::convexHull(cv::Mat(contours[largest_contour]), hull[0], false);
          // cv::drawContours(contour, hull, 0, cv::Scalar(255, 0, 0), 3);
          
          if (hull[0].size() > 2)
          {
              std::vector<int> hullIndexes;
              cv::convexHull(cv::Mat(contours[largest_contour]), hullIndexes, true);
              std::vector<cv::Vec4i> convexityDefects;
              cv::convexityDefects(cv::Mat(contours[largest_contour]), hullIndexes, convexityDefects);
              cv::Rect boundingBox = cv::boundingRect(hull[0]);
              // cv::rectangle(contour, boundingBox, cv::Scalar(255, 0, 0));
              cv::Point center = cv::Point(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
              std::vector<cv::Point> validPoints;
              for (size_t i = 0; i < convexityDefects.size(); i++)
              {
                  cv::Point p1 = contours[largest_contour][convexityDefects[i][0]];
                  cv::Point p2 = contours[largest_contour][convexityDefects[i][1]];
                  cv::Point p3 = contours[largest_contour][convexityDefects[i][2]];
                  double angle = std::atan2(center.y - p1.y, center.x - p1.x) * 180 / CV_PI;
                  double inAngle = innerAngle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
                  double length = std::sqrt(std::pow(p1.x - p3.x, 2) + std::pow(p1.y - p3.y, 2));
                  if (angle > -30 && angle < 160 && std::abs(inAngle) > 20 && std::abs(inAngle) < 120 && length > 0.1 * boundingBox.height)
                  {
                      validPoints.push_back(p1);
                  }
              }
              for (size_t i = 0; i < validPoints.size(); i++)
              {
                  cv::circle(contour, validPoints[i], 9, cv::Scalar(255, 0, 0), 2);
              }

              // Average number of fingers over past NUM_FRAMES_FINGER_AVG frames so that we don't get spurious outputs
              if(curr_num_finger_frames >= NUM_FRAMES_FINGER_AVG) {
                // Calculating std dev of recorded fingers in last NUM_FRAMES_FINGER_AVG frames to see if it was stable enough to call appropriate Gantry function
                avg_num_fingers = (float)total_count_fingers / (float)NUM_FRAMES_FINGER_AVG;
                double finger_sq_sum = inner_product(fingers_in_frame.begin(), fingers_in_frame.end(), fingers_in_frame.begin(), 0.0);
                finger_stdev = sqrt(finger_sq_sum / NUM_FRAMES_FINGER_AVG - avg_num_fingers * avg_num_fingers);
                
                // cout << "Standard dev of fingers = " << finger_stdev << endl;
                // cout << max_circle << endl;

                // fingerToGesture(round(avg_num_fingers), max_circle.center, contour.rows, contour.cols);
                // Call Gantry function only if the std dev is low enough
                if(finger_stdev < NUM_FINGERS_STD_DEV) {
                  // cout << "Avg number of fingers = " << round(avg_num_fingers) << endl;
                  fingerToGesture(round(avg_num_fingers), max_circle.center, contour.rows, contour.cols);
                }

                // Reset all counters/variables for next block of NUM_FRAMES_FINGER_AVG frames regardless
                curr_num_finger_frames = 0;
                total_count_fingers = 0;
                fingers_in_frame.clear();
              } else {
                curr_num_finger_frames++;
                total_count_fingers += validPoints.size();
                fingers_in_frame.push_back(validPoints.size());
                
                putText(contour, "Num fingers: " + to_string(avg_num_fingers), Point(0, 25), FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1, LINE_AA);
                putText(contour, "StdDev:" + to_string(finger_stdev), Point(0,50), FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1, LINE_AA);
              }
          }
      }
    

    // Resizing window to monitor (1280 x 1024)
    scale_width = screen_res_width / contour.cols;
    scale_height = screen_res_height / contour.rows;
    scale = min(scale_width, scale_height);

    resized_window_width = int(contour.cols * scale);
    resized_window_height = int(contour.rows * scale);

    //cv2.WINDOW_NORMAL makes the output window resizealbe
    namedWindow("Hand Detection", WINDOW_NORMAL);
 
    //resize the window according to the screen resolution
    resizeWindow("Hand Detection", resized_window_width, resized_window_height);

    imshow("Hand Detection", contour);

    int key = cv::waitKey(1);

    listener_HAND.release(frames_HAND);
    // listener_GRIP.release(frames_GRIP);
  }

  dev_HAND->stop();
  dev_HAND->close();

  // dev_GRIP->stop();
  // dev_GRIP->close();


  delete registration_HAND;
  // delete registration_GRIP;
  return  0;
}
