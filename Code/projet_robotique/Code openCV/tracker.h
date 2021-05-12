using namespace cv;
using namespace std;

int* getColor(VideoCapture cap);
void deplacementServoMoteur(Point p, int xCentre, int yCentre, serial_com *sp);
Point calculBarycentre (Mat hsv, int h, int s, int v);
void CallBackFunc(int event, int x, int y, int flags, void* ptr);