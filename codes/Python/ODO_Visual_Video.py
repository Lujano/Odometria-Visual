""""
///////////////////////////////////////////////////////////////////////
//                                                                   //
//                             Vslam        v1                       //
//                        Luis  Lujano, 13-10775                     //
//                       Jaime Villegas, 13-11493                    //
///////////////////////////////////////////////////////////////////////
"""



import cv2
import numpy as np
import matplotlib.pyplot as plt
plt.ion()



fIdx = 57

import odotools as odotools

file_directory = "../../../Datasets/kitti/odometry/poses"
dataset = '../../../Datasets/kitti/odometry/00/image_2'
odotools.read_poses( file_directory, "00", fIdx)
gt_poses = np.zeros((0, 2), dtype=np.float)  # (x, y) relative camera coordinates

i = 1
poses = np.zeros((0, 2), dtype=np.float)  # (x, y) relative camera coordinates
t_p = np.zeros(3).transpose()

despl = np.zeros(3).transpose()

fs = cv2.FileStorage("../C++/ODO/Output_fast_complete.yaml", cv2.FILE_STORAGE_READ)
fn = fs.getNode("Trayectoria")
data = np.asanyarray(fn.mat())


while True:

    im1_path = '%s/%06d.png' % (dataset, fIdx)
    im2_path = '%s/%06d.png' % (dataset, fIdx + 1)
    im1 = cv2.imread(im1_path)
    im2 = cv2.imread(im2_path)
    if fIdx == 500:
        plt.waitforbuttonpress()
    if im1 is None:
        raise Exception("File %s does not exist" % im0_path)
    while im2 is None:
        fIdx = fIdx + 1
        i = i + 1
        print("Imagen no encontrada:{}".format(fIdx))
        im2_path = '%s/%06d.png' % (dataset, fIdx + 1)
        im2 = cv2.imread(im2_path)
    # Compute R, t from absolute scale and essential matrix
    # Compute R, t from absolute scale and essential matrix
    scale = odotools.getAbsoluteScale(i + 1)
    t_p = t_p+scale*np.abs(np.transpose(data[i]))
    poses = np.vstack([poses, [t_p[0], t_p[2]]])  # x, y
    gt_poses = odotools.gt_poses[1:len(poses)+1, 0:2]

    # Compute errors
    # odo_dist = np.sqrt(np.sum(np.diff(poses, axis=0)**2))
    t_error = np.mean(np.sqrt(np.sum((gt_poses - poses)**2, axis=1)))

    # Plot trajectory and GT
    plt.figure(1)
    plt.clf()
    plt.subplot(2, 1, 1)
    plt.title('Camera')
    plt.imshow(im1[..., ::-1])
    plt.subplot(2, 1, 2)
    plt.plot(poses[:, 0], poses[:, 1], marker='o', c='b', label="Visual Odometry")
    plt.plot(gt_poses[:, 0], gt_poses[:, 1], marker='s', c='g', label="GPS")
    ##plt.hold()
    plt.axis('equal')
    plt.title("Frame %d, Cum. err. %.2fm" % (fIdx, t_error))
    plt.legend()

    plt.draw()
    plt.waitforbuttonpress(0.005)

    fIdx += 1
    i += 1