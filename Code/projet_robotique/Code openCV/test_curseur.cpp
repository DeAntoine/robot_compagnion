#include <iostream>
#include </usr/local/Cellar/opencv/4.1.2/include/opencv4/opencv2/opencv.hpp>
 
using namespace std;
using namespace cv;
 
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
}
 
int main()
{
    // Read image from file 
    Mat img = imread("chat.jpg");
 
    //if fail to read the image
    if ( img.empty() ) 
    { 
        cout << "Error loading the image" << endl;
        return -1; 
    }
 
    //Create a window
    namedWindow("ImageDisplay", 1);
 
    //set the callback function for any mouse event
    setMouseCallback("ImageDisplay", CallBackFunc, NULL);
 
    //show the image
    imshow("ImageDisplay", img);
 
    // Wait until user press some key
    waitKey(0);
 
    return 0;
 
}