
#include "opencv2/highgui.hpp"
//Ejemplo3

int g_slider_position = 0;
CvCapture* g_capture = NULL;

void onTrackbarSlide(int pos){
    cvSetCaptureProperty(
        g_capture,
        CV_CAP_PROP_POS_FRAMES,
        pos
    );
}
int main(int argc, char** argv)
{
    cvNamedWindow("Example3", CV_WINDOW_AUTOSIZE);
    g_capture = cvCreateFileCapture(argv[1]);
    float frames = (float)cvGetCaptureProperty( // Numero de frames en el video
        g_capture,
        CV_CAP_PROP_POS_FRAMES
    );
    if (frames!= 0){
        cvCreateTrackbar(
            "Position",
            "Example3",
            &g_slider_position,
            frames,
            onTrackbarSlide
        );
    }
    IplImage* frame;

    while(1){
        frame = cvQueryFrame(g_capture);
        if(!frame) break;
        cvShowImage("Example3", frame);
        char c = cvWaitKey(33); // espera 33 ms para que el usuario presione una tecla
        if (c == 27) break; // esc key
    }
    cvReleaseCapture(&g_capture);
    cvDestroyWindow("Example3");

    return 0;
}
