/*
#include "./libfreenect2/include/libfreenect2/libfreenect2.hpp"
#include "./libfreenect2/include/libfreenect2/frame_listener_impl.h"
#include "./libfreenect2/include/libfreenect2/registration.h"
#include "./libfreenect2/include/libfreenect2/packet_pipeline.h"
#include "./libfreenect2/include/libfreenect2/logger.h"
*/

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>


#include <iostream>
#include <vector>
#include <cmath>

using namespace std;




int main(int argc, char **argv) {

  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;

  if(freenect2.enumerateDevices() == 0)
  {
    cout << "no device connected!" << endl;
    return -1;
  }
  
  string serial = freenect2.getDefaultDeviceSerialNumber();

  pipeline = new libfreenect2::CpuPacketPipeline();


  dev = freenect2.openDevice(serial, pipeline);

  if (dev == 0)
  {
	cerr << "No device opened!"<<endl;
	return -1;
  }

  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;
  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);



  dev->start();
  cout << "device serial: " << dev->getSerialNumber() << endl;
  cout << "device firmware: " << dev->getFirmwareVersion() << endl;

  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

  while(1)
  {
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    registration->apply(rgb, depth, &undistorted, &registered);

    listener.release(frames);
  }

  dev->stop();
  dev->close();
 
  cout << "Hello world" << endl;
  return  0;
}
