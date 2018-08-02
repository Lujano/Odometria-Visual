#include <stdio.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include "opencv2/core.hpp"

using namespace std;
using namespace cv;


void get_gps_data(Mat &gps_odometry){
    ifstream inFile;
    Mat point = Mat::zeros(1, 2, CV_64F);
    //gps_odometry.push_back(point);
    inFile.open("00_gt.txt");
    if (!inFile) {
        cerr << "Unable to open file datafile.txt";
        exit(1);   // call system to stop
    }
    string x;
    while (!inFile.eof()) {
        getline(inFile, x);
        if (x.size()!= 0){
            std::vector<std::string> v;
            split( x, v, ' ' ); // size 12
            //double a = atof(v[3].c_str());
            //cout<<v[3]<<endl;
            point.at<double>(0,0) =  atof(v[3].data()); //x
            point.at<double>(0,1) =  atof(v[11].data()); //x
            gps_odometry.push_back(point);
        }
      
    

    }
    inFile.close();
}

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

// g++ -g -o GPSreader.out GPSreader.cpp `pkg-config opencv --cflags --libs`