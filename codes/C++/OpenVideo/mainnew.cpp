#include "opencv2/opencv.hpp"
#include <iostream>
#include <cstdio>
#include <ctime>
using namespace cv;

int main()
{   
    clock_t start;
    double duration;
    start = clock();
    /* Your algorithm here */
    VideoCapture cap(0); // open the default camera
    /*
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    while(duration < 5.0){
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    }
    */

  
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat edges;
    namedWindow("edges",1);
    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        cvtColor(frame, edges, COLOR_BGR2GRAY);
        GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        Canny(edges, edges, 0, 30, 3);
        imshow("edges", edges);
        char c_input = (char) waitKey(25);
       if( c_input == 'q' | c_input == ((char)27) ) return 0; // Mostrar la imagen hasta que se presione una teclas
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}