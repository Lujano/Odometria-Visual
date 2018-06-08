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

int main(int argc, char** argv)
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

    string file_directory = parser.get<string>("@image"); // Direccion del directorio con imagenes
    int index1_int;
    string index1_str, index2_str;
    Mat src, src2;
    
    index1_int = 57 ; // Primera imagen a leer

    while(index1_int < 1049){ // penultima imagen  a leer
        index1_str = file_directory + patch::to_string(index1_int)+".png";
        index2_str = file_directory + patch::to_string(index1_int+1)+".png";
        src = imread(index1_str, 1); // Cargar con color, flag = 1
        src2 = imread(index2_str, 1);
       


        while ( src2.empty() & index1_int < 1049) // En caso de lectura de una imagen no existe
         {
            index1_int = index1_int+1;// Saltar a la siguiente imagen
            cout<< "Imagen no encontrada:"<< index1_int<< endl;
            index2_str = file_directory + patch::to_string(index1_int+1)+".png";
            src2 = imread(index2_str, 1);
        }
        Mat gray1, gray2;
        cvtColor(src, gray1, CV_BGR2GRAY);
        cvtColor(src2, gray2, CV_BGR2GRAY);
        imshow("source1", gray1);
        imshow("source2", gray2);
        char c_input = (char) waitKey(25);
        if( c_input == 'q' | c_input == ((char)27) ) return 0; // Mostrar la imagen hasta que se presione una teclas
        cout<< index1_int<< endl; 
        index1_int ++; // Siguiente par de imagenes
       
    }
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "elapsed time = "<< elapsed_secs << endl;
    return 0;
}

