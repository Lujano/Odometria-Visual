#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/core/types.hpp"
#include <opencv2/plot.hpp>

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

void sColorBalance(Mat input_img, Mat &output_img);
void readme();
/** @function main */
int main( int argc, char** argv )
{
cv::CommandLineParser parser(argc, argv,
        "{help h||}{@image|../data/pic1.png|}"
    );
    if (parser.has("help"))
    {
        help();
        return 0;
    }


    clock_t begin = clock(); // Tiempo de inicio del codigo

    string file_image= parser.get<string>("@image"); // Direccion del directorio con imagenes
    Mat src, out;
    
    src = imread(file_image, 1); // Cargar con color, flag = 1

    //imshow("src", src);
    //char c_input = (char) waitKey(-1);
    //if( c_input == 'q' | c_input == ((char)27) ) return 0; // Mostrar la imagen hasta que se presione una teclas

    
    sColorBalance(src, out);
  
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "elapsed time = "<< elapsed_secs << endl;
    return 0;
}



static void help()
{
    cout << "\nThis program demonstrates a simple SLAM.\n"
            "Usage:\n"
            "./main.out <directory_name>, Default is ../data/pic1.png\n" << endl;
}

void sColorBalance(Mat input_img, Mat &output_img){
    Mat channels[3];
    split(input_img, channels);
    int i = 0;
    
    
    std::cout << channels[0].at<Vec>(0)<< endl;




}