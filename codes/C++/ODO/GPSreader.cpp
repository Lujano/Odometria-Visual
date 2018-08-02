#include <stdio.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include "opencv2/core.hpp"

using namespace std;
using namespace cv;

size_t split(const std::string &txt, std::vector<std::string> &strs, char ch)
{
    size_t pos = txt.find( ch );
    size_t initialPos = 0;
    strs.clear();

    // Decompose statement
    while( pos != std::string::npos ) {
        strs.push_back( txt.substr( initialPos, pos - initialPos ) );
        initialPos = pos + 1;

        pos = txt.find( ch, initialPos );
    }

    // Add the last one
    strs.push_back( txt.substr( initialPos, std::min( pos, txt.size() ) - initialPos + 1 ) );

    return strs.size();
}

void get_gps_data(Mat &gps_odometry, int first_index){
    ifstream inFile;
    Mat point = Mat::zeros(1, 2, CV_64F);
    Mat gps_data;
    double first_x;
    double first_y;
    
    //gps_odometry.push_back(point);
    inFile.open("00_gt.txt");
    if (!inFile) {
        cerr << "Unable to open file datafile.txt";
        exit(1);   // call system to stop
    }
    string x;
    std::vector<std::string> v;
    
    while (!inFile.eof()) {
        getline(inFile, x);

        if (x.size()!= 0){
            
            split( x, v, ' ' ); // size 12
            //double a = atof(v[3].c_str());
            //cout<<v[3]<<endl;
            point.at<double>(0,0) =  atof(v[3].data()); //x
            point.at<double>(0,1) =  -atof(v[11].data()); //x
            gps_data.push_back(point);
        }
      

    }
    // Optimizar esta parte con un solo while
    first_x = gps_data.at<double>(first_index,0); //x
    first_y = gps_data.at<double>(first_index,1); //y

    for (int i =0; i < gps_data.rows; i= i+1){

        point.at<double>(0,0) = (gps_data.at<double>(i,0)-first_x); //x
        point.at<double>(0,1) = (gps_data.at<double>(i,1)-first_y); //y
        gps_odometry.push_back(point);
    }

    cout<<gps_odometry.at<double>(2,0)<<endl;
    cout<<gps_odometry.at<double>(2,1)<<endl;
    cout<< gps_data.rows<<endl;
    cout<< "first point" <<first_x<< ","<< first_y<< endl;
    inFile.close();
}
// g++ -g -o GPSreader.out GPSreader.cpp `pkg-config opencv --cflags --libs`