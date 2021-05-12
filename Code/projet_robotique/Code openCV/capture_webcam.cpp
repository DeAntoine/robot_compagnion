#include </usr/local/Cellar/opencv/4.1.2/include/opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;


void CallBackFunc(int event, int x, int y, int flags, void* ptr);

int main(int, char**)
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat frame;
    namedWindow("MyCam",1);

    Point p;
    p.x = -1;
    p.y = -1;

    setMouseCallback("MyCam", CallBackFunc, &p);
  
    cap.set(CAP_PROP_FRAME_WIDTH,320);  //taille de la fenetre
    cap.set(CAP_PROP_FRAME_HEIGHT,240); //au dela de 320*240
    int compteur = 0, b=0, g=0, r=0;

    while(1){

        if(cap.read(frame)){// get a new frame from camera
            compteur++;
            imshow("MyCam", frame);
            if(p.x != -1 && p.y != -1)
            {
                int sommeb = 0, sommeg = 0, sommer = 0;
                //printf("x : %d / y : %d\n", p.x, p.y);
                int w=frame.cols;
                int h=frame.rows;            
                
                for(int i=-2; i<=2; i++)
                {
                    for(int j=-2; j<=2; j++) 
                    {
                        b = (int)frame.at<Vec3b>(p.y,p.x)[0];
                        g = (int)frame.at<Vec3b>(p.y,p.x)[1];
                        r = (int)frame.at<Vec3b>(p.y,p.x)[2];
                        sommeb += b;
                        sommeg += g;
                        sommer += r;
                    }
                }
                b = sommeb/25;
                g = sommeg/25;
                r = sommer/25;

                Mat image(50, 50, CV_8UC3, Scalar(b, g, r));
                imshow("image", image);
                waitKey(0);
                p.x = -1; 
                p.y = -1;
            }
            
        }
    
        if(waitKey(30) >= 0) break;
    }
    
    return 0;
}


void CallBackFunc(int event, int x, int y, int flags, void* ptr)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        Point *p = (Point*)ptr;
        p->x = x;
        p->y = y;
    }
}