#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include "serial.h"

using namespace cv;
using namespace std;

int* getColor(VideoCapture cap);
void CallBackFunc(int event, int x, int y, int flags, void* ptr);

int main(int, char**)
{

	char c;
    serial_com *sp = (serial_com *) malloc(sizeof(serial_com));
    serial_open(sp, "/dev/cu.usbmodem1421201");	
    
    VideoCapture cap(1); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat frame;
	Mat thr, gray, src, mask1, mask2, hsv, res;


    serial_write('i', sp);
    
    int *selectedColor = getColor(cap);
    int h = selectedColor[0];
    int s = selectedColor[1];
    int v = selectedColor[2];


int xCentre = frame.cols/2, yCentre = frame.rows/2;
cout << "xcentre :" <<xCentre;
cout << "ycentre :" <<yCentre;
int nbErreurs = 1;
while(1) {

 	// Capture frame-by-frame
    cap >> src;

	//Converting image from BGR to HSV color space.
        cvtColor(src, hsv, COLOR_BGR2HSV);

	int a = 0, b = 0, count = 1;

	for (int i=0; i < hsv.rows;i++) {
		for (int j=0; j < hsv.cols;j++) {
			if (hsv.at<Vec3b>(i,j)[0] >= h-20 && hsv.at<Vec3b>(i,j)[0] <= h+20 && hsv.at<Vec3b>(i,j)[1] >= s-50 && hsv.at<Vec3b>(i,j)[1] <= s+50 && hsv.at<Vec3b>(i,j)[2] >= v-50 && hsv.at<Vec3b>(i,j)[0] <= v+50) {
				a += i;
				b += j;
				count++;
			}
			else {
				hsv.at<Vec3b>(i,j)[0] = hsv.at<Vec3b>(i,j)[1] = hsv.at<Vec3b>(i,j)[2] = 0;
			}
		}
	}

	a = a/count;
	b = b/count;

	Point p(b,a);

	if (b > xCentre && a > yCentre) { //En bas à droite
		serial_write('b', sp);
		serial_write('d', sp);
        cout << "bd" << endl;
	}
	else if (b > xCentre && a < yCentre) { //En haut à droite
		serial_write('h', sp);
		serial_write('d', sp);
        cout << "hd" << endl;
	}
	else if (b < xCentre && a > yCentre) { //En bas à gauche
		serial_write('b', sp);
		serial_write('g', sp);
        cout << "bg" << endl;
	}
	else if (b < xCentre && a < yCentre) { //En haut à gauche
		serial_write('h', sp);
		serial_write('g', sp);
        cout << "hg" << endl;
	}
	 
	// show the image with a point mark at the centroid
	circle(src, p, 5, Scalar(0,0,0), -1);
	imshow("Image with center",src);
	imshow("Image hsv",hsv);
    int erreur = sqrt(pow((xCentre-b),2) + pow((yCentre-a), 2));
    if (b > xCentre) {
        erreur = -erreur;
    }
    //cout << nbErreurs << " " << erreur << endl;
    nbErreurs++;
	waitKey(30);
}
  serial_close(sp);  
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

void serial_open(serial_com *sp, char *name){
    sp->fd = open(name, O_RDWR);
    sp->nomDuPort = (char *) malloc(20*sizeof(char));
    strcpy(sp->nomDuPort, name);
}

void serial_close(serial_com *sp){
    close(sp->fd);
}

void serial_read(serial_com *sp){
    char buf;
    int n;
    while (1) 
    {
        read(sp->fd, &buf, 1);
        printf("%c\n ",buf);
    }
}

void serial_write(char txt, serial_com *sp)
{
        write(sp->fd, &txt, 1);
}

int* getColor(VideoCapture cap) {
    Mat frame, hsv;

    namedWindow("MyCam",1);

    Point p;
    p.x = -1;
    p.y = -1;

    setMouseCallback("MyCam", CallBackFunc, &p);
  
    cap.set(CAP_PROP_FRAME_WIDTH,320);  //taille de la fenetre
    cap.set(CAP_PROP_FRAME_HEIGHT,240); //au dela de 320*240
    int h=0, s=0, v=0;
    int sommeh = 0, sommes = 0, sommev = 0;

    while(1){

        if(cap.read(frame)){// get a new frame from camera
            imshow("MyCam", frame);
		    waitKey(30);
	        cvtColor(frame, hsv, COLOR_BGR2HSV);

            if(p.x != -1 && p.y != -1)
            {           
                for(int i=-2; i<=2; i++)
                {
                    for(int j=-2; j<=2; j++) 
                    {
                        h = (int)hsv.at<Vec3b>(p.y+i,p.x+j)[0];
                        s = (int)hsv.at<Vec3b>(p.y+i,p.x+j)[1];
                        v = (int)hsv.at<Vec3b>(p.y+i,p.x+j)[2];
                        sommeh += h;
                        sommes += s;
                        sommev += v;
                    }
                }
                h = sommeh/25;
                s = sommes/25;
                v = sommev/25;

                break;
            }
            
        }
    }

    int* answer = (int*) malloc(3*sizeof(int));
    *answer = h;
    *(answer+1) = s;
    *(answer+2) = v;
    return answer;
}