""""
///////////////////////////////////////////////////////////////////////
//                                                                   //
//                             Vslam        v1                       //
//                        Luis  Lujano, 13-10775                     //
//                       Jaime Villegas, 13-11493                    //
///////////////////////////////////////////////////////////////////////
"""



import numpy as np
import matplotlib.pyplot as plt
plt.ion()


fIdx = 57
fline = fIdx-57

i = 0

while True:



    i+=1

    gt_poses = np.loadtxt("GPS_data.txt")

    # senod2 = R_p[0,1]

    #print(R_p)
    #print(R2)

    # if np.sign(senod) != np.sign(senod2):
    #     t_p = abs(t_p)
    #     print("hola")

    # Stack up the poses

    # Compute errors
    # odo_dist = np.sqrt(np.sum(np.diff(poses, axis=0)**2))

    # Plot trajectory and GT
    plt.figure(2)
    plt.clf()
    plt.plot(gt_poses[:, 0], gt_poses[:, 1], marker='s', c='g', label="GroundTruth")
    plt.axis('equal')
    plt.title("Frame %d\n" % (fIdx))
    plt.legend()

    plt.draw()
    plt.waitforbuttonpress(0.02)

    fIdx += 1
