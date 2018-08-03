import cv2
import yaml
import numpy as np
import matplotlib.pyplot as plt
import cv2
fs = cv2.FileStorage("Output_ODOV_00000004_00000001.yaml", cv2.FILE_STORAGE_READ)

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

plt.figure("ODOMETRIA VISUAL by LUJANO, Detector = {}, Matcher = {}".format(_detector, _matcher))
plt.subplot(3, 1, 1)
plt.plot(Trayectoria_ODOV[:, 0], Trayectoria_ODOV[:, 1], c='b', label="Visual Odometry")
plt.plot(Trayectoria_GPS[:, 0], Trayectoria_GPS[:, 1], c='g', label="GroundTruth")
plt.xlabel("x(m)")
plt.ylabel("y(m)")
xmin = Trayectoria_GPS[:, 0][np.argmin(Trayectoria_GPS[:, 0])]
xmax = Trayectoria_GPS[:, 0][np.argmax(Trayectoria_GPS[:, 0])]
ymin = Trayectoria_GPS[:, 1][np.argmin(Trayectoria_GPS[:, 1])]
ymax = Trayectoria_GPS[:, 1][np.argmax(Trayectoria_GPS[:, 1])]
#plt.xlim(xmin, xmax)
#plt.ylim(ymin, ymax)
plt.legend()
plt.title("Fps= {:.3f}, Puntos = {:.2f}".format(FPS_mean, Npoints_mean))
plt.subplot(3, 1, 3)
plt.plot(np.arange(int((Trayectoria_ERROR.size)/2.0)), Trayectoria_ERROR[:, 0],marker='o', c='b', label="ERROR DIST")
plt.title("Despl = {:.3f} m, Drift = {:.3f} m".format(Dist_recorrida, error_sum))
plt.xlabel("x(m)")
plt.ylabel("y(m)")
plt.legend()

plt.show()