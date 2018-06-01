#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <ctime> // Libreria para medir tiempo
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

static void help()
{
    cout << "\nThis program demonstrates a simple SLAM.\n"
            "Usage:\n"
            "./main.out <directory_name>, Default is ../data/pic1.png\n" << endl;
}

int main()
{
    clock_t begin = clock(); // Tiempo de inicio del codigo
    string file_directory = "../../../../Datasets/kitti/odometry/00/image_2/";
    int index1_int;
    string index1_str, index2_str;
    Mat src, src2;
    
    index1_int = 57 ; // Primera imagen a leer

    while(index1_int < 1049){ // penultima imagen  a leer
        index1_str = file_directory + patch::to_string(index1_int)+".png";
        index2_str = file_directory + patch::to_string(index1_int+1)+".png";
        src = imread(index1_str, 1); // Cargar con color, flag = 1
        src2 = imread(index2_str, 1);
     

        while ( src2.empty() & index1_int < 1049)
         {
            index1_int = index1_int+1;// Saltar a la siguiente imagen
            cout<< "Imagen2 no encontrada:"<< index1_int<< endl;
            index2_str = file_directory + patch::to_string(index1_int+1)+".png";
            src2 = imread(index2_str, 1);
        }
        
        imshow("source1", src);
        imshow("source2", src2);
        waitKey(1);
        cout<< index1_int<< endl; 
        index1_int ++; // Siguiente par de imagenes
       
    }
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "elapsed time = "<< elapsed_secs << endl;
    return 0;
}

