#include <opencv2/opencv.hpp>
#include <math.h> 
#include <stdio.h>

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
    //cap.set(CV_CAP_PROP_FRAME_WIDTH, 256); 
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 256); 

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
        GaussianBlur(edges, edges, Size(9,9), 4.0);
        //Canny(edges, edges, 0, 30, 3);
        findContours(edges, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        cout<<"num of objects: "<< contours.size()<<endl;
        for (int i = 0; i < contours.size(); i++) {
            vector<Point> pts = contours.at(i);
            cout << "num of vertecies: "<< pts.size()<<endl;
            //double angle = atan((pts.at(3).y-pts.at(2).y)/(pts.at(3).x-pts.at(2).x));
            cout<<"Object "<<i<<" has points:"<<endl;
            for (int j = 0; j < pts.size(); j++)
                cout<<"Point "<<j<<" is at: "<< pts.at(j)<<endl;
            drawContours(drawing, contours, i, cv::Scalar(255, 255, 255));
        }
        
    
        
        imshow("drawing", drawing);
        waitKey(1);
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}