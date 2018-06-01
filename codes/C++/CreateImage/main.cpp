
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace std;
int main(int argc, char** argv)
{
    IplImage* img = cvLoadImage(argv[1]);
    cvNamedWindow("Example1", 0);
    cvShowImage("Example1", img);
    cvWaitKey(2000);
    cvReleaseImage(&img);
    cvDestroyWindow("Example1");

    return 0;
}
