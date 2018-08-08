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
#include "GPSreader.cpp"
#include <math.h>

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

enum Detector
{
    USE_KAZE,
    USE_AKAZE,
    USE_ORB,
    USE_SIFT,
    USE_SURF,
    USE_FAST
};

enum Matcher
{
    USE_BRUTE_FORCE,
    USE_FLANN
};

namespace patch
{
    template < typename T > std::string to_string( const T& n ) // funcion para transformar entero en string
    {
        std::ostringstream stm ;
        stm << n ;
        if (n < 10){
            
            return "0000000"+stm.str();
        }
        if (n < 100){
            
            return "000000"+stm.str();
        }
        if (n >= 100 & n<1000){
            return "00000"+stm.str();
        }
            
        return "0000"+stm.str() ;
    }
}
  
static void help();
void readme();
int Odometry(Mat img_1, Mat img_2, Mat &R, Mat &t, Mat &imageOut, int detector, int matcher, double *Npoints, double *Npairs);
void ROI(vector<KeyPoint> &points, Mat descriptors, int thresholdx, int thresholdy, int w, int h, vector<KeyPoint> &pointsOut,  Mat &desOut); //verifica si el punto se encuentra en una ventana dada
void checkStatus(vector<Point2f> &points, vector<uchar> &status, vector<Point2f> &pointsOut);
void get_matcher(int _matcher);
void get_detector(int _detector);

Ptr<Feature2D> detector;                    //!< Pointer to OpenCV feature extractor
Ptr<DescriptorMatcher> matcher; //!< Pointer to OpenCV feature Matcher
/** @function main */
int main( int argc, char** argv )
{
const String keys =
    "{help h usage ? |      | print this message   }"
    "{directory     |      | data input directory} "
    "{detector      |      | detector to use  }"
    "{matcher       |       | matcher to use   }"
    "{first_frame      |      | first_frame index  }"
    "{last_frame      |      | first_frame index  }";
cv::CommandLineParser parser(argc, argv, keys);
    if (parser.has("help"))
    {
        help();
        return 0;
    }


    string file_directory = parser.get<string>("directory"); // Direccion del directorio con imagenes
    int _detector = parser.get<int>("detector");
    int _matcher = parser.get<int>("matcher");
    int index1_int, index2_int;
    string index1_str, index2_str;
    Mat src, src2;
    int first_frame = 1;
    int first_frame_int = parser.get<int>("first_frame");// Primera imagen a leer
    
    index1_int = first_frame_int; // Primera imagen a leer
    int index_last_int =  parser.get<int>("last_frame");; //Ultima imagen a leer
    int break_prcs =0;

    Mat odov; // Matriz vacia de vectores de longitud 2 (comienza con 00)
    Mat gps;
    Mat error;
    Mat point_odov = Mat::zeros(Size(1, 2), CV_64F);
    Mat point_gps = Mat::zeros(Size(1, 2), CV_64F);
    Mat point_error = Mat::zeros(Size(1, 2), CV_64F);
    Mat R, t; // matriz de rotacion y traslacion
    Mat R_p ; // matriz de rotacion temporal
    Mat traslation; 
    Mat plot_x;
    Mat plot_y;
    Mat img_fast;
    Mat ground_truth;
    double scale;
    get_gps_data(ground_truth, index1_int);
    get_matcher(_matcher);
    get_detector(_detector);

    double error_sum = 0;
    double FPS_sum = 0;
    double FPS_mean = 0;
    double Npoints = 0;
    double Npoints_sum = 0;
    double Npoints_mean = 0;
    double Npairs = 0;
    double Npairs_sum = 0;
    double Npairs_mean = 0;
    double Dist_recorrida = 0;

    VideoWriter outputVideo;
    outputVideo.open("Ouput_ODOV_"+patch::to_string(_detector)+"_"+patch::to_string(_matcher)+".mp4", VideoWriter::fourcc('M','J','P','G'), 15.0, Size(640, 600), true);
    Mat Data_test = Mat::zeros(Size(1, 9), CV_64F); //detector, descriptor, first_frame, last_frame, Dist_recorrida, Drift, FPS_mean, Nro de  puntos mean
    while(index1_int < index_last_int){ // penultima imagen  a leer
        clock_t begin = clock(); // Tiempo de inicio del codigo
        index1_str = file_directory + "camera_left.image_raw_"+ patch::to_string(index1_int)+".pgm";
        index2_int = index1_int+1;
        index2_str = file_directory +"camera_left.image_raw_"+ patch::to_string(index2_int)+".pgm";
        src = imread(index1_str,0); // Cargar con color, flag = 1
        src2 = imread(index2_str, 0);

        while ( src2.empty() & index1_int < index_last_int) // En caso de lectura de una imagen no existe
         {
            index1_int = index1_int+1;// Saltar a la siguiente imagen
            cout<< "Imagen no encontrada:"<<patch::to_string(index1_int)+".pgm"<< endl;
            index2_int = index1_int+1;
            index2_str = file_directory + "camera_left.image_raw_"+patch::to_string(index2_int)+".pgm";
            src2 = imread(index2_str, 0);
        }
        //Mat gray1, gray2;
        //cvtColor(src, gray1, CV_BGR2GRAY);
        //cvtColor(src2, gray2, CV_BGR2GRAY);

        // ODOV
        break_prcs = Odometry(src, src2, R, t, img_fast, USE_SURF, USE_FLANN, &Npoints, &Npairs);

        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        FPS_sum = (FPS_sum+1.0/elapsed_secs);
        Npoints_sum = Npoints_sum+Npoints;
        Npairs_sum = Npairs_sum+Npairs;
       

        //odometry.push_back(despl)
        //cout<< "Matrix R="<< R<<endl;
        //cout<< "Matrix t="<< t<<endl;
        double desp_x = ground_truth.at<double>(index2_int,0)-ground_truth.at<double>(index1_int,0);
        double desp_y = ground_truth.at<double>(index2_int,1)-ground_truth.at<double>(index1_int,1);
        if (first_frame == 1){
            
            scale = sqrt(pow(desp_x,2)+pow(desp_y,2 ));
            Mat traslation2align= (Mat::eye(Size(3, 3), CV_64F))*t*scale;
            Point2f vector_gps, vector_odov;
            vector_gps.x =  ground_truth.at<double>(index2_int,0);
            vector_gps.y =  ground_truth.at<double>(index2_int,1);
            // Normalizacion
            vector_gps.x =  vector_gps.x/sqrt(pow(vector_gps.x,2)+pow(vector_gps.y,2));
            vector_gps.y =  vector_gps.y/sqrt(pow(vector_gps.x,2)+pow(vector_gps.y,2));
            
            vector_odov.x = traslation2align.at<double>(0,0);
            vector_odov.y = traslation2align.at<double>(0,2);
            vector_odov.x =  vector_odov.x/sqrt(pow(vector_odov.x,2)+pow(vector_odov.y,2));
            vector_odov.y =  vector_odov.y/sqrt(pow(vector_odov.x,2)+pow(vector_odov.y,2));
            float cross_product = vector_odov.x*vector_gps.y-vector_odov.y*vector_gps.x;
            float dot_product = vector_odov.x*vector_gps.x+vector_odov.y*vector_gps.y;
             R_p = Mat::zeros(3, 3, CV_64F);
             
             R_p.at<double>(0,0) = dot_product;
             R_p.at<double>(0,2) = -cross_product;
             R_p.at<double>(2,0) = cross_product; // Error de años porq la componente y esta en la 3 fila :/
             R_p.at<double>(2,2) = dot_product;
             R_p.at<double>(1,2) = 1;
             
            //R_p = Mat::eye(3, 3, CV_64F);
             
            /*
            R_p.at<double>(0,2) = 1/sqrt(2);
             R_p.at<double>(0,1) = -1/sqrt(2);
             R_p.at<double>(2,0) =  1/sqrt(2);
             R_p.at<double>(2,2) = 1/sqrt(2);;
             R_p.at<double>(1,1) = 1;
             */
            /*
            Mat v_matrix=  Mat::zeros(3, 3, CV_64F);
            v_matrix.at<double>(0,1) = v3;
            v_matrix.at<double>(1,0) = -v3;
            R_p = (Mat::eye(Size(3, 3), CV_64F))+v_matrix+(v_matrix*v_matrix)*1.0/(1.0+dot_product);
            */
            traslation = Mat::zeros(3, 1, CV_64F);
            /*
            cout <<R_p<<endl;
            cout << "Vertor_GPS"<<vector_gps<<endl;
            cout << "Vector ODOV"<<vector_odov<<endl;
            */
            first_frame = 0;
        }
        else{
            /*
            t = Mat::zeros(Size(1, 3), CV_64F);
            t.at<double>(0,0) = 1;
            R = Mat::eye(Size(3, 3), CV_64F);
            */
            //cout << "R ="<<R<<endl;
            scale = sqrt(pow(desp_x,2)+pow(desp_y,2 ));
            traslation = traslation +R_p*t*scale;
            R_p = R*R_p;
            
        }
        // PUSH Results
        point_odov.at<double>(0,0) = traslation.at<double>(0,0); //x
        point_odov.at<double>(0,1) = traslation.at<double>(0,2); //y
        
        point_gps.at<double>(0,0) = ground_truth.at<double>(index1_int,0); //x
        point_gps.at<double>(0,1) = ground_truth.at<double>(index1_int,1); //y

        point_error.at<double>(0,0) = sqrt(pow(point_odov.at<double>(0,0)-point_gps.at<double>(0,0),2)+pow(point_odov.at<double>(0,1)-point_gps.at<double>(0,1), 2 )); // error de distancia
        error_sum = error_sum+point_error.at<double>(0,0);
        point_error.at<double>(0,1) = 0; // error de orientacion

        odov.push_back(point_odov);
        gps.push_back(point_gps);
        error.push_back(point_error);

        plot_x.push_back(ground_truth.at<double>(index1_int,0));
        plot_y.push_back(ground_truth.at<double>(index2_int,1));
        //cout<<"Translation"<<traslation<<endl;
        plot_x.push_back(traslation.row(0));
        plot_y.push_back(traslation.row(2)); // Y es la tercera fila

        // Plot data
        double min_x, max_x, min_y, max_y, min, max;
        minMaxLoc(plot_x, &min_x, &max_x);
        minMaxLoc(plot_y, &min_y, &max_y);
        min = min_x;
        max = max_x;
        if (min_y < min_x )min = min_y;
        if (max_y > max_x )max = max_y;

        Mat plot_result;
        Ptr<plot::Plot2d> plot = plot::createPlot2d(plot_x, plot_y);
        plot->setPlotBackgroundColor( Scalar( 0, 0, 0 ) );
        plot->setNeedPlotLine(0);
        plot->setPlotAxisColor (Scalar( 255, 0, 0 ) );
        plot->setMaxY (max+10);
        plot->setMinY (min-10);
        plot->setMaxX (max+10);
        plot->setMinX (min-10);
        plot->render( plot_result );
    
        
        FPS_mean = FPS_sum/(plot_x.rows/2.0);
        Npairs_mean = Npairs_sum/(plot_x.rows/2.0);
        Npoints_mean = Npoints_sum/(plot_x.rows/2.0);
        Dist_recorrida = Dist_recorrida+scale;
        cout << "Distancia recorrida = "<<Dist_recorrida<< " m"<<endl;
        cout << "FPS promedio = "<< FPS_mean << endl;
        cout << "Drift = "<< error_sum<< " m"<<endl;
        cout <<"Nro puntos promedio = "<<Npoints_mean<<endl;
        cout <<"Nro parejas promedio = "<<Npairs_mean<<endl;
        cout <<"Nro imagenes procesadas = "<<plot_x.rows/2<<endl;
        cout<<endl;
         // Operaciones 
           
        

          //imshow("Puntos detectados FAST-1", img_keypoints_1 );
         
        //namedWindow("Puntos detectados FAST-1_bajo analisis",WINDOW_NORMAL);
        //resizeWindow("Puntos detectados FAST-1_bajo analisis", 300, 300);
        Mat img_res1, img_res2;
        resize(img_fast,img_res1, Size( 640,300));//resize image
        resize(plot_result,img_res2, Size(640, 300));//resize image
        Mat img_win;
        vconcat(img_res1, img_res2, img_win);
        imshow( "ODOMETRIA VISUAL by LUJANO", img_win);
        
        outputVideo << img_win;
        
        
        //cout<< "Traslation" << traslation<< endl;

        //cout<< "Indice =" << index1_int<< endl; 
         if (break_prcs == 1)
            break; // Mostrar la imagen hasta que se presione una teclas
            
        index1_int ++; // Siguiente par de imagenes
            
       
    }
    
    Data_test.at<double>(0,0) = double(_detector);
    Data_test.at<double>(0,1) = double(_matcher); 
    Data_test.at<double>(0,2) = double(first_frame_int);
    Data_test.at<double>(0,3) = double(index2_int); // last_frame
    Data_test.at<double>(0,4) = Dist_recorrida;
    Data_test.at<double>(0,5) = error_sum;
    Data_test.at<double>(0,6) = FPS_mean;
    Data_test.at<double>(0,7) = Npoints_mean;
    Data_test.at<double>(0,7) = Npairs_mean;


    FileStorage file1("Output_ODOV_"+patch::to_string(_detector)+"_"+patch::to_string(_matcher)+".yaml", FileStorage::WRITE); // Archivo para guardar el desplazamiento 
    file1 << "Trayectoria_ODOV"<< odov;
    file1 << "Trayectoria_GPS"<< gps;
    file1 << "Trayectoria_ERROR"<< error;
    file1 << "Data_test"<< Data_test;
    
    return 0;
}



static void help()
{
    cout << "\nThis program demonstrates a simple SLAM.\n"
            "Usage:\n"
            "./main.out <directory_name>, Default is ../data/pic1.png\n" << endl;
}
int Odometry(Mat img_1, Mat img_2, Mat &R, Mat &t, Mat &imageOut, int _detector, int _matcher, double *Npoints,double *Npairs){
 clock_t begin = clock(); // Tiempo de inicio del codigo
  int w1, h1; // Image size
  w1 = img_1.size().width;
  h1 = img_1.size().height;
  //-- Paso 1: detectar los puntos clave utilizando SIFT
  
  

    
  vector<KeyPoint> keypoints_1, keypoints_2, keypoints_roi; // Vector para almacenar los puntos detectados con FAST
  Mat descriptors_1, descriptors_2, descriptors_roi;
    // Acotar ventana en la imagen 1
  int thresholdx = 60, thresholdy = 60;
  w1 = img_1.size().width;
  h1 = img_1.size().height;
  Mat mask = Mat::zeros(img_1.size(), img_1.type());
  Mat roi(mask, cv::Rect(thresholdx, thresholdy, w1-2*thresholdx,h1-2*thresholdy)); // second parameter size
  roi = Scalar(255, 255, 255);
  detector -> detectAndCompute( img_1, mask, keypoints_1, descriptors_1 );
  detector -> detectAndCompute( img_2, Mat(), keypoints_2, descriptors_2 );


  vector< vector<DMatch> > matches; // Cambio
  matcher-> knnMatch( descriptors_1, descriptors_2, matches, 2 );
  double nn_match_ratio = 0.8f; // Nearest-neighbour matching ratio
  vector<KeyPoint> matched1, matched2;
  for(unsigned i = 0; i < matches.size(); i++) {
      if(matches[i][0].distance < nn_match_ratio * matches[i][1].distance) {
          matched1.push_back(keypoints_1[matches[i][0].queryIdx]);
          matched2.push_back(keypoints_2[matches[i][0].trainIdx]);
      }
  }


cout << "Keypoint"<<matches.size()<<"rows"<<descriptors_roi.size()<<" cols ="<<descriptors_1.cols<<endl;
  //cout<< "Descriptors"<< keypoints_roi.data()<<endl;



  //-- Paso 4: Calcular la matriz Esencial
    // Parametros intrisecos de la camara
  double fx, fy, focal, cx, cy;
  /*
  fx = 9.842439e+02;
  fy = 9.808141e+02;
  cx =  6.900000e+02;
  cy =  2.331966e+02;
  */
  fx = 7.188560000000e+02;

  fy = 7.188560000000e+02;
  cx =  6.071928000000e+02;
  cy =  1.852157000000e+02;
  
  focal = fx;
  Mat E; // matriz esencial

  std::vector<Point2f> points1_OK, points2_OK; // Puntos finales bajos analisis
  vector<int> point_indexs;

  cv::KeyPoint::convert(matched1, points1_OK,point_indexs);
  cv::KeyPoint::convert(matched2, points2_OK,point_indexs);
   *Npoints = double(keypoints_1.size());
   *Npairs = double(matched1.size());
    
  E = findEssentialMat(points1_OK, points2_OK, focal, Point2d(cx, cy), RANSAC, 0.999, 1.0, noArray());
 
  //--Paso 5: Calcular la matriz de rotación y traslación de puntos entre imagenes 
   int p;
   p = recoverPose(E, points1_OK, points2_OK, R, t, focal, Point2d(cx, cy), noArray()   );

  clock_t end = clock(); // Tiempo de inicio del codigo
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Tiempo loop"<<elapsed_secs<<endl;
  
//-- Paso 6: Draw keypoints
  Mat img_keypoints_1, img_keypoints_2, img_keypointsOK; 

  
  drawKeypoints( img_1, matched1, img_keypointsOK, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

  imageOut = img_keypointsOK;
  
  char c_input = (char) waitKey(1);
  if( c_input == 'q' | c_input == ((char)27) ) 
    return 1;
  return 0;


}
  /** @function readme */

  void readme()
  { std::cout << " Usage: ./Fast <img1> <img2>" << std::endl; }

  void  ROI(vector<KeyPoint> &points,Mat descriptors,  int thresholdx, int thresholdy, int w, int h,vector<KeyPoint> &pointsOut, Mat &desOut){
        KeyPoint point; // Punto final condicionado al umbral
        int px, py; // Puntos temporales
        for (std::vector<KeyPoint>::const_iterator i = points.begin(); i != points.end(); ++i)
        { 
          px = (*i).pt.x;
          py = (*i).pt.y;
          if( (py < (h-thresholdy)) & (py > thresholdy)){
              desOut.push_back(descriptors.row(0));
              point.pt.x =  px;
              point.pt.y = py;
              pointsOut.push_back(point); // Anexar punto umbral al vector de puntos de salida
          }
    
          
        }
    }


    void checkStatus(vector<Point2f> &points, vector<uchar> &status, vector<Point2f> &pointsOut){
      uchar status_ok;
      std:vector<Point2f>::const_iterator pointerData;
      pointerData = points.begin();
      for (std::vector<uchar>::const_iterator i = status.begin(); i != status.end(); ++i)
        {
          status_ok = *i;
          if (status_ok == 1){
            pointsOut.push_back(*pointerData); // Anexar el punto con status OK al vector de salida
          }
          pointerData++;
        }
    }

    void get_matcher(int _matcher){
    // select input matcher
        switch (_matcher)
        {
        case USE_BRUTE_FORCE:
        {
            cout<<"Descriptor: BRUTE_FORCE"<<endl;
            matcher = BFMatcher::create();
            break;
        }
        case USE_FLANN:
        {   
            cout<<"Descriptor: FLANN"<<endl;
            matcher = FlannBasedMatcher::create();
            break;
        }
        default:
        {
            cout<<"Descriptor: FLANN"<<endl;
            matcher = FlannBasedMatcher::create();
            break;
        }
        }
        }
    void get_detector(int _detector){
        switch (_detector)
            {
            case USE_KAZE:
            {
                cout<<"Detector: KAZE"<<endl;
                detector = KAZE::create();
                break;
            }
            case USE_AKAZE:
            {
                cout<<"Detector: AKAZE"<<endl;
                detector = AKAZE::create();
                break;
            }
            case USE_SIFT:
            {
                cout<<"Detector: SIFT"<<endl;
                detector = SIFT::create();
                break;
            }
            case USE_SURF:
            {
                cout<<"Detector: SURF"<<endl;
                detector = SURF::create(400);
                break;
            }
            case USE_ORB:
            {
                cout<<"Detector: ORB"<<endl;
                detector = ORB::create();
                break;
            }
            default:
            {
                cout<<"Detector: KAZE"<<endl;
                detector = KAZE::create();
                break;
            }
            }
    }
//  g++ -g -o Visual_ODO_Features2.out Visual_ODO_Features2.cpp `pkg-config opencv --cflags --libs`
// ./Visual_ODO_Features.out -directory=../../../../../../../media/victor/CAB21993B219855B/Datasets/00/ -detector=0 -matcher=0 -first_frame=30 -last_frame=4539
//./Visual_ODO_Features.out -directory=../../../../Datasets/00/00.txt.d/ -detector=0 -matcher=0 -first_frame=30 -last_frame=4539

// https://gitlab.com/srrg-software/srrg_proslam/tree/master
// https://stackoverflow.com/questions/29407474/how-to-understand-kitti-camera-calibration-files