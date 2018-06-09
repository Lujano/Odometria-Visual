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

/** @function main */
int main( int argc, char** argv )
{
  if( argc != 3)
  { readme(); return -1; }

  Mat img_1 = imread( argv[1], IMREAD_GRAYSCALE );
  Mat img_2 = imread( argv[2], IMREAD_GRAYSCALE );
  if( !img_1.data || !img_2.data)
  { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

  //-- Step 1: Detect the keypoints using SURF Detector
  int threshold=10;
  bool nonmaxSuppression=true;
  int type=FastFeatureDetector::TYPE_9_16;

  Ptr< FastFeatureDetector> detector_1 = 	FastFeatureDetector::create(threshold ,nonmaxSuppression , type);
  Ptr< FastFeatureDetector> detector_2 = 	FastFeatureDetector::create(threshold ,nonmaxSuppression , type);
  // Parametros flujo optico
  Size winSize=Size(21, 21);
  TermCriteria crit = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
  int maxLevel=3;
  int flags=0;
  double minEigThreshold=1e-4;
  Ptr< SparsePyrLKOpticalFlow > sparse = SparsePyrLKOpticalFlow::create(winSize, maxLevel, crit, flags ,minEigThreshold );
  vector<KeyPoint> keypoints_1, keypoints_2;

  detector_1 -> detect( img_1, keypoints_1 );
  detector_2-> detect( img_2, keypoints_2 );
  
  
  // Calcular flujo optico
  vector<Point2f> points_f, points_out;
  vector<int> point_indexs;
  cv::KeyPoint::convert(keypoints_1,points_f,point_indexs);
  vector<uchar> status;
  vector<float> err;
  sparse-> calc(img_1, img_2, points_f, points_out, status, err) ;

  
  // Compute descriptors
  Mat descriptors;

  //detector_2->compute(img_2, keypoints_2, descriptors_2);


//-- Draw keypoints
  Mat img_keypoints_1, img_keypoints_2; 

  drawKeypoints( img_1, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  drawKeypoints( img_2, keypoints_2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

  imshow("Puntos detectados FAST-1", img_keypoints_1 );
  imshow("Puntos detectados FAST-2", img_keypoints_2 );
  waitKey(0);
  return 0;
}
  /** @function readme */
  void readme()
  { std::cout << " Usage: ./SURF_descriptor <img1> <img2>" << std::endl; }