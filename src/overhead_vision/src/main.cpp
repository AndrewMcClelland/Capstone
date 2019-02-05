#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int, char**)
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat edges;
    namedWindow("edges",1);
    for(;;)
    {
        Mat frame;
        Mat thresh;
        vector<cv::Vec4i> hierarchy;
        vector<vector<Point> > contours;
        
        cap >> frame; // get a new frame from camera

        Mat drawing = Mat::zeros( frame.size(), CV_8UC3 );

        cvtColor(frame, edges, COLOR_BGR2GRAY);
        GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        Canny(edges, edges, 0, 30, 3);
        findContours(edges, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contours.size(); i++) 
            drawContours(drawing, contours, i, cv::Scalar(255, 255, 255));
        
        imshow("drawing", drawing);
        waitKey(1);
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}