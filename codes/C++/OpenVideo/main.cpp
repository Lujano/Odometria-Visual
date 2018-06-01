
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace std;
int main(int argc, char** argv)
{
    cvNamedWindow("Example2", CV_WINDOW_AUTOSIZE);
    CvCapture* capture = cvCreateFileCapture(argv[1]);
    IplImage* frame;

    while(1){
        frame = cvQueryFrame(capture);
        if(!frame) break;
        cvShowImage("Example2", frame);
        char c = cvWaitKey(33); // espera 33 ms para que el usuario presione una tecla
        if (c == 27) break; // esc key
    }
    cvReleaseCapture(&capture);
    cvDestroyWindow("Example2");

    return 0;
}
