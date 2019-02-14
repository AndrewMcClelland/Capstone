#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include "opencv2/videoio/videoio_c.h"
#include <math.h> 
#include <stdio.h>
#include <libfreenect.h>

using namespace cv;
using namespace std;

class object
{
public:
    double angle;
    int x;
    int y;
    int width;
    int height;

};

int main(int, char**)
{
    VideoCapture cap(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 800); 
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 600); 

    if(!cap.isOpened())  // check if we succeeded
        return -1;

    vector<object> object_list;

    Mat edges;


    int iLowH = 0;
    int iHighH = 179;

    int iLowS = 0; 
    int iHighS = 255;

    int iLowV = 0;
    int iHighV = 255;

  
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);




    for(;;)
    {
        Mat frame;
        

        Mat imgHSV;
        Mat imgThresholded;

        vector<cv::Vec4i> hierarchy;
        vector<vector<Point> > contours;
        
        cap >> frame; // get a new frame from camera


        cvtColor(frame, imgHSV, COLOR_BGR2HSV);
        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); 

        GaussianBlur(edges, edges, Size(9,9), 4.0);
        Canny(edges, edges, 0, 30, 3);
        findContours(edges, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        vector<vector<Point> > contours_poly( contours.size() );
        
        imshow("drawing", drawing);
        waitKey(1);
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}