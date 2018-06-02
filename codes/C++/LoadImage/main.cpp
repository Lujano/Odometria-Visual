#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

static void help()
{
    cout << "\nThis program demonstrates line finding with the Hough transform.\n"
            "Usage:\n"
            "./houghlines <image_name>, Default is ../data/pic1.png\n" << endl;
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
    string filename = parser.get<string>("@image");
    if (filename.empty())
    {
        help();
        cout << "no image_name provided" << endl;
        return -1;
    }
    Mat src = imread(filename, 0);
    if(src.empty())
    {
        help();
        cout << "can not open " << filename << endl;
        return -1;
    }
    int height = src.cols;
    int width = src.rows;
    cout<< "Tamaño"<< src.size()<< "Height ="<< height<< "Rows" << width<<endl;
    imshow("source", src);
    cv::ROI
    waitKey();

    return 0;
}
