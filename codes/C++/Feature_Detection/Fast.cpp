#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/core/types.hpp"
using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

void readme();
void thresholdTrans(vector<KeyPoint> &points, int thresholdy, int w, int h, vector<KeyPoint>  &pointsOut); //verifica si el punto se encuentra en una ventana dada
void thresholdRot(vector<KeyPoint> &points, int thresholdx, int w, int h, vector<KeyPoint> &pointsOut); //verifica si el punto se encuentra en una ventana dada
void checkStatus(vector<Point2f> &points, vector<uchar> &status, vector<Point2f> &pointsOut);

/** @function main */
int main( int argc, char** argv )
{
  if( argc != 3)
  { readme(); return -1; }

  Mat img_1 = imread( argv[1], IMREAD_GRAYSCALE );
  Mat img_2 = imread( argv[2], IMREAD_GRAYSCALE );
  if( !img_1.data || !img_2.data)
  { std::cout<< " --(!) Error reading images " << std::endl; return -1; }
  
  int w1, h1; // Image size
  w1 = img_1.size().width;
  h1 = img_1.size().height;
  
  
  //-- Paso 1: detectar los puntos clave utilizando Fast Features
  int threshold=50;
  bool nonmaxSuppression=true;
  int type=FastFeatureDetector::TYPE_9_16;

  Ptr< FastFeatureDetector> detector_1 = 	FastFeatureDetector::create(threshold ,nonmaxSuppression , type);
  
  vector<KeyPoint> keypoints_1; // Vector para almacenar los puntos detectados con FAST
  detector_1 -> detect( img_1, keypoints_1 );
  cout << "Puntos detectados = "<< keypoints_1.size()<< endl;

  //-- Paso 2: Definir la region de interes (ROI)
  vector<KeyPoint> key_points_ROT_out, key_points_TRANS_out, keypointsOK;
  int thresholdx = 40, thresholdy = 50;
  thresholdRot(keypoints_1, thresholdx, w1, h1, key_points_ROT_out); // Puntos a considerar para la rotacion
  thresholdTrans(key_points_ROT_out, thresholdy, w1, h1,key_points_TRANS_out); // Puntos a considerar para la traslacion
  
  keypointsOK = key_points_TRANS_out; // Puntos resultantes bajo analisis
  cout << "Puntos bajo analisis ="<< keypointsOK.size()<< endl;

  //-- Paso 3: Calcular flujo optico entre las dos imagenes
  Size winSize=Size(21, 21);
  TermCriteria crit = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 40, 0.001);
  int maxLevel=3;
  int flags=0;
  double minEigThreshold=1e-3;
  Ptr< SparsePyrLKOpticalFlow > sparse = SparsePyrLKOpticalFlow::create(winSize, maxLevel, crit, flags ,minEigThreshold );

    // Convertir formato Keypoint a Point2f
  vector<Point2f> keypointsOK1_f, keypoints2Out_f;
  vector<int> point_indexs;
  cv::KeyPoint::convert(keypointsOK, keypointsOK1_f,point_indexs);

    // Calcular flujo optico
  vector<uchar> status; // 	output status vector (of unsigned chars); each element of the vector is set to 1 if the flow for
                        //   the corresponding features has been found, otherwise, it is set to 0. 
  vector<float> err;
  sparse-> calc(img_1, img_2, keypointsOK1_f, keypoints2Out_f, status, err) ;

    // Considerar solo parejas de puntos (status = 1)
  vector<Point2f> points1_OK, points2_OK; // Puntos finales bajos analisis
  checkStatus(keypointsOK1_f, status, points1_OK); // points 1 y 2 tienen mismo tamaño (igual que ...
  checkStatus( keypoints2Out_f, status, points2_OK);  // vector status)
  cout << "Puntos emparejados = "<< points1_OK.size()<< endl;

  //-- Paso 4: Calcular la matriz Esencial
    // Parametros intrisecos de la camara
  double fx, fy, focal, cx, cy;
  fx = 7.188560000000e+02;
  fy = 7.188560000000e+02;
  cx =  6.071928000000e+02;
  cy =  1.852157000000e+02;
  focal = fx;
  Mat E; // matriz esencial
  E = findEssentialMat(points1_OK, points2_OK, focal, Point2d(cx, cy), RANSAC, 0.999, 1.0, noArray());

  //--Paso 5: Calcular la matriz de rotación y traslación de puntos entre imagenes 
   int p;
   Mat R, t; // matriz de rotación y traslación
   p = recoverPose(E, points1_OK, points2_OK, R, t, focal, Point2d(cx, cy), noArray()   );
   cout<< "Matrix R="<< R<<endl;
   cout<< "Matrix t="<< t<<endl;

   Mat traslation;
   traslation = R*t;
   FileStorage file1("Output_fast.yaml", FileStorage::WRITE);
   cout<< "Traslation" << traslation<< endl;

  file1 << "Rotation"<< R;

//-- Paso 6: Draw keypoints
  Mat img_keypoints_1, img_keypoints_2, img_keypointsOK; 

  drawKeypoints( img_1, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  drawKeypoints( img_1, keypointsOK, img_keypointsOK, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
 

  imshow("Puntos detectados FAST-1", img_keypoints_1 );
  imshow("Puntos detectados FAST-1_bajo analisis", img_keypointsOK );
  waitKey(0);
  return 0;
}
  /** @function readme */
  void readme()
  { std::cout << " Usage: ./Fast <img1> <img2>" << std::endl; }

  void  thresholdTrans(vector<KeyPoint> & points, int thresholdy, int w, int h, vector<KeyPoint> &points_out){
        KeyPoint point; // Punto final condicionado al umbral
        int px, py; // Puntos temporales
        for (std::vector<KeyPoint>::const_iterator i = points.begin(); i != points.end(); ++i)
        { 
          px = (*i).pt.x;
          py = (*i).pt.y;
          if( (py < (h-thresholdy)) & (py > thresholdy)){
            point.pt.x =  px;
            point.pt.y = py;
            points_out.push_back(point); // Anexar punto umbral al vector de puntos de salida
          }
    
          
        }
    }
  void thresholdRot(vector<KeyPoint>  &points, int thresholdx, int w, int h, vector<KeyPoint>  &points_out)
    {
        KeyPoint point; // Punto final condicionado al umbral
        int px, py; // Puntos temporales
        for (std::vector<KeyPoint>::const_iterator i = points.begin(); i != points.end(); ++i)
        { 
          px = (*i).pt.x;
          py = (*i).pt.y;
          if( (px < (w-thresholdx)) & (px > thresholdx)){
          point.pt.x=  px;
          point.pt.y = py;
          points_out.push_back(point); // Anexar punto umbral al vector de puntos de salida
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