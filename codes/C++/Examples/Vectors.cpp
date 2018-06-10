#include <iostream>
#include <vector>
#include <typeinfo>
#include <opencv2/highgui.hpp>      //for imshow
using namespace std;
using namespace cv;

int main(int argc, char const *argv[])
{   

    vector<float> a ( 2, 1.0); // Vector de dos elementos iguales a dos
    Point2f p;
    p.x = 0.2;
    p.y = 0.3;
    //vector<double> b(a); // Copia del a
    printf("%6.*lf\n", 4, a[0]); // Imprimir flotante
    //cout << "Vector b = "<< b[0]<<endl;
    printf("Punto : x = %6.*lf, y = %6.*lf ", 4, p.x, 4,  p.y);

    // Declarar un vector de puntos 2f
    vector<Point2f> vpoints;
    vector<float> vacio;
    vacio.push_back(2.00) ; // Insertar un valor al vector vacio
    Point2f point;
    point.x = 2.0;
    point.y = 3.0;
    vpoints.push_back(point);
    printf(" Vacio = %6.*lf\n", 4, vacio[0]); // Imprimir flotante con precision 4
    printf("Vector = [");
    for (std::vector<float>::const_iterator i = a.begin(); i != a.end(); ++i)
       printf("%6.*lf ", 4, *i);
    printf("]");
    cout<< "vpoints"<< (*vpoints.begin()).x << endl;
    return 0;
}
