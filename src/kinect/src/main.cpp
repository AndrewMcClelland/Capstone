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

  libfreenect2::SyncMultiFrameListener listener(  libfreenect2::Frame::Color
						| libfreenect2::Frame::Ir
						| libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;

  dev->setColorFrameListener(&listener);
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

    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
    Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
    Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);

    //imshow("rgb", rgbmat);
    imshow("ir", irmat / 4096.0f);
    imshow("depth", depthmat / 4096.0f);

    registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

    Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
    Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
    Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);


    imshow("undistorted", depthmatUndistorted / 4096.0f);
    imshow("registered", rgbd);
    imshow("depth2RGB", rgbd2 / 4096.0f);
    int key = cv::waitKey(1);
    listener.release(frames);
  }

  dev->stop();
  dev->close();
  

  delete registration; 
  return  0;
}
