#include "opencv2/opencv.hpp"
#include <iostream>
#include <cstdio>
#include <string>

using namespace cv;
using namespace std;

namespace patch
{
    template < typename T > std::string to_string( const T& n ) // funcion para transformar entero en string
    {
        std::ostringstream stm ;
        stm << n ;
        if (n < 100){
            return "0000"+stm.str();
        }
        if (n >= 100 & n<1000){
            return "000"+stm.str();
        }
            
        return "00"+stm.str() ;
    }
}
static void help();

int main( int argc, char** argv )
{
    cv::CommandLineParser parser(argc, argv,
        "{help h||}{@video|../data/video.mp4|}"
    );
    if (parser.has("help"))
    {
        help();
        return 0;
    }

    string video_name= parser.get<string>("@video"); // Direccion del directorio con imagenes
    int index1_int;


    VideoCapture video_in; 


    video_in.open(video_name);

    int n_frames=	static_cast<int>(video_in.get(CV_CAP_PROP_FRAME_COUNT));
    double	rate =	video_in.get(CV_CAP_PROP_FPS);

    //printf("%d", n_frames);
    cout<<"Numero de frames = "<<n_frames<<endl; 
    cout<<"fps = "<<rate<<endl; 

    if(!video_in.isOpened())  // check if we succeeded
        return -1;

    Mat edges;
    namedWindow("Video",1);

    int c_frame= 0;
    for(c_frame = 0; c_frame < n_frames; c_frame++) // Leer todos los frames del video
    {
        Mat frame;
        video_in >> frame; // get a new frame from camera
       // imshow("Video",frame);
        imwrite("Output/frame_"+patch::to_string(c_frame)+".jpg", frame);
        //char c_input = (char) waitKey(25);
      // if( c_input == 'q' | c_input == ((char)27) ) return 0; // Mostrar la imagen hasta que se presione una teclas
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}


// Implementacion de funciones
static void help()
{
    cout << "\nThis program demonstrates a simple SLAM.\n"
            "Usage:\n"
            "./main.out <directory_name>, Default is ../data/video.mp4\n" << endl;
}