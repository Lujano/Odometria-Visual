import cv2
import yaml
import numpy as np
import matplotlib.pyplot as plt
import os
import argparse

def detector2string(_detector):
    if _detector == 0.0 : return "KAZE"
    if _detector == 1.0 : return "AKAZE"
    if _detector == 2.0 : return "ORB"
    if _detector == 3.0 : return "SIFT" 
    if _detector == 4.0 : return "SURF"
    if _detector == 5.0 : return "FAST"

def matcher2string(_matcher):
    if _matcher == 0.0 : return "BRUTE FORCE"
    if _matcher == 1.0 : return "FLANN"

def main():
    #Constuccion del parse y del argumento
    ap = argparse.ArgumentParser()
    ap.add_argument("-f", "--file", required = True, help = "Archivo YAML de Entrada")
    args = vars(ap.parse_args())
    fs = cv2.FileStorage(args["file"], cv2.FILE_STORAGE_READ)
    fn = fs.getNode("Trayectoria_ODOV")
    Trayectoria_ODOV = np.asanyarray(fn.mat())
    Trayectoria_ODOV =Trayectoria_ODOV.reshape(int((Trayectoria_ODOV.size)/2.0), 2)

    fn = fs.getNode("Trayectoria_GPS")
    Trayectoria_GPS = np.asanyarray(fn.mat())
    Trayectoria_GPS =Trayectoria_GPS.reshape(int((Trayectoria_GPS.size)/2.0), 2)

    fn = fs.getNode("Trayectoria_ERROR")
    Trayectoria_ERROR= np.asanyarray(fn.mat())
    Trayectoria_ERROR=Trayectoria_ERROR.reshape(int((Trayectoria_ERROR.size)/2.0), 2) 

    fn = fs.getNode("Data_test")
    Data_test = np.asanyarray(fn.mat())
    Data_test = Data_test.reshape(Data_test.size,1) 

    _detector = Data_test[0][0]
    _matcher = Data_test[1][0]
    first_frame_int = Data_test[2][0]
    last_frame_int = Data_test[3][0]
    Dist_recorrida = Data_test[4][0]
    error_sum = Data_test[5][0]
    FPS_mean = Data_test[6][0]
    Npoints_mean = Data_test[7][0]

    print("Detector Usado = {}, Macther Usado = {}".format(_detector, _matcher))
    print("Primer Frame = {}, Ultimo Frame = {}".format(first_frame_int, last_frame_int))
    print("Distancia recorrida = {:.3f} m, Drift = {:.3f} m".format(Dist_recorrida, error_sum))
    print("Fps Promedio= {:.3f}, Nro de puntos detectados promedio = {:.3f}".format(FPS_mean, Npoints_mean))
    #print(fn.mat())

    fig =plt.figure("ODOMETRIA VISUAL by LUJANO_{}_{}".format(detector2string(_detector),matcher2string(_matcher)))
    #plt.subplot(3, 1, 1)
    ax = fig.add_subplot(2, 1, 1)
    plt.scatter(0, 0,marker = '^', c='black', label="Origen", s = 300)
    plt.plot(Trayectoria_ODOV[:, 0], Trayectoria_ODOV[:, 1], marker = 'o', c='b', label="Visual Odometry")
    plt.plot(Trayectoria_GPS[:, 0], Trayectoria_GPS[:, 1],marker = 'o', c='g', label="Ground Truth")
    

    plt.xlabel("x(m)")
    plt.ylabel("y(m)")
    """
    xmin = Trayectoria_ODOV[:, 0][np.argmin(Trayectoria_ODOV[:, 0])]
    xmax = Trayectoria_ODOV[:, 0][np.argmax(Trayectoria_ODOV[:, 0])]
    ymin = Trayectoria_ODOV[:, 1][np.argmin(Trayectoria_ODOV[:, 1])]
    ymax = Trayectoria_ODOV[:, 1][np.argmax(Trayectoria_ODOV[:, 1])]
    minor_ticks_x = np.arange(xmin-(xmax-xmin)/20.0, xmax+(xmax-xmin)/10.0, (xmax-xmin)/50.0)
    major_ticks_x = np.arange(xmin, xmax+(xmax-xmin)/10.0, (xmax-xmin)/10.0)
    minor_ticks_y = np.arange(ymin-(ymax-ymin)/10.0, ymax+(ymax-ymin)/10.0, (ymax-ymin)/50.0)
    major_ticks_y = np.arange(ymin, ymax+(ymax-ymin)/10.0, (ymax-ymin)/10.0)
    ax.set_xticks(major_ticks_x)
    ax.set_xticks(minor_ticks_x, minor=True)
    ax.set_yticks(major_ticks_y)
    ax.set_yticks(minor_ticks_y, minor=True)
    ax.grid(which='both')
    ax.grid(which='minor', alpha=0.5)
    ax.grid(which='major', alpha=0.8)
    """
    plt.legend()
    plt.title(" {}-{}, Fps Prom= {:.3f}, Ptos dect prom = {:.2f}, Dist recorrida = {:.2f} m".format(detector2string(_detector), matcher2string(_matcher),FPS_mean, Npoints_mean, Dist_recorrida))
    
 
    #plt.xlim(xmin, xmax)
    #plt.ylim(ymin, ymax)
    
    ax2 = fig.add_subplot(2, 1, 2)
    plt.plot(np.arange(int((Trayectoria_ERROR.size)/2.0)), Trayectoria_ERROR[:, 0], c='r', label="ERROR DIST")
    plt.title("Dist recorrida = {:.3f} m, Drift = {:.3f} m, Nro imágenes = {}".format(Dist_recorrida, error_sum,last_frame_int-first_frame_int+1 ))
    plt.ylabel("Error Dist(m)")
    plt.xlabel("frame")
    plt.legend()
    
    plt.show()



if __name__ == '__main__':
    main()