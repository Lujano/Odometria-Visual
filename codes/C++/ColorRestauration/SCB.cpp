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
#include "sort.cpp"

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
void readme();
int findmax(Mat Img );
int findmin(Mat Img );
void sat_max(Mat Img, int max );
void sat_min(Mat Img, int min );
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
    Mat src;
    Mat channels[3];
    Mat temp_channels[3];
    src = imread(file_image, 1); // Cargar con color, flag = 1

    Mat grises(300, 200, CV_8U, 127); // Imagen en escala de grises
    Mat color(300, 200, CV_8UC3, Scalar(200, 0, 200)); // Imagen en escala de grises
    Mat grises2 ;
    color.convertTo(grises2, CV_8U, 1.0, 0.0);
    /*
    namedWindow("Img Grises");
    namedWindow("Img1");
    imshow("Img Grises", color);
    imshow("Img1", src);
    */
    split(src, channels);
    char c_input = (char) waitKey(0);
    //if( c_input == 'q' | c_input == ((char)27) ) return 0; // Mostrar la imagen hasta que se presione una teclas

    int max;
    int min;
    int i;
    vector<int> array(src.rows*src.cols);
    Mat array_1d;
    float porcentaje = 1.0;
    float mitad_porcentaje = porcentaje/200.0;
    int Vmin;
    int Vmax;
    Mat sorted;
    for (i = 0; i < 3; i++){
       
        array_1d = channels[i].reshape(0, 1);
        //array.assign(array_1d.datastart, array_1d.dataend);
        //int *array2 = &array[0];
        
        cv::sort(array_1d, sorted, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
        //_introsort(array2, array.size());
        Vmin = int(array.size()*mitad_porcentaje);
        Vmax = int(array.size()*(1-mitad_porcentaje));
        max = sorted.at<uchar>(0, Vmax);
        min = sorted.at<uchar>(0, Vmin);
        sat_max(channels[i], max);
        sat_min(channels[i], min);


        normalize(channels[i], channels[i], 0, 255, NORM_MINMAX );
        
        cout << "Channel"<<i<<endl;
        cout << "Max = "<< max << ", Min = "<< min<< endl;
        cout << "size " << array_1d.cols<<endl;
        
    }
    
    Mat output_img;
    merge(channels, 3, output_img);
    imwrite("blue.jpg",output_img);
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

int findmax(Mat Img ){
    int i, j;
    int max = 0;
    int value;
    for (i = 0 ; i< Img.rows;i++ ){
        for (j= 0; j < Img.cols; j++){
            value= Img.at<uchar>(i, j);
            if (value> max) max = value;     
        }
    }
    return max;
}

int findmin(Mat Img ){
    int i, j;
    int min = 255;
    int value;
    for (i = 0 ; i< Img.rows;i++ ){
        for (j= 0; j < Img.cols; j++){
            value= Img.at<uchar>(i, j);
            if (value< min) min = value;     
        }
    }
    return min;
}

void sat_max(Mat Img, int max ){
    int i, j;
    int value;
    for (i = 0 ; i< Img.rows;i++ ){
        for (j= 0; j < Img.cols; j++){
            value= Img.at<uchar>(i, j);
            if (value>max) Img.at<uchar>(i, j) = max;     
        }
    }

}
void sat_min(Mat Img, int min ){
    int i, j;
    int value;
    for (i = 0 ; i< Img.rows;i++ ){
        for (j= 0; j < Img.cols; j++){
            value= Img.at<uchar>(i, j);
            if (value< min) Img.at<uchar>(i, j) = min;     
        }
    }

    
}