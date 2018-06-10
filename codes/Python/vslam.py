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

def thresholdTrans(img, thresholdy, w, h): # verifica si el punto se encuentra en una ventana dada
    imgy = img[:,1]
    # print(w)
    # print(h)
    # print("sd")

    valid = (imgy <(h-thresholdy)) &(imgy > thresholdy) # se asumen traslaciones en y
    index = np.where(valid <= 0)
    imgPoints = np.delete(img, index, 0)

    return imgPoints


def thresholdRot(img, thresholdx, w, h): # verifica si el punto se encuentra en una ventana dada

    imgx = img[:,0]
    # print(w)
    # print(h)
    # print("sd")

    valid = (imgx <(w-thresholdx)) & (imgx > thresholdx) # se asumen rotaciones en direccion x
    index = np.where(valid <= 0)
    imgPoints = np.delete(img, index, 0)

    return imgPoints

#Stanford deep learning, berkley
fIdx = 57
dataset = '../../../Datasets/kitti/odometry/00/image_2'
# dataset = '../ressources/kitti/odometry/02/image_2'  # OR use that database

import odotools as odotools
odotools.read_poses(dataset + "/../../poses", "00", fIdx)
gt_poses = np.zeros((0, 2), dtype=np.float)  # (x, y) relative camera coordinates

poses = np.zeros((0, 2), dtype=np.float)  # (x, y) relative camera coordinates
R_p, t_p = None, None

while True:
    im0_path = '%s/%06d.png' % (dataset, fIdx)
    im1_path = '%s/%06d.png' % (dataset, fIdx+1)
    im1 = cv2.imread(im0_path)
    im2 = cv2.imread(im1_path)
    if fIdx == 500:
        plt.waitforbuttonpress()
    if im1 is None:
        raise Exception("File %s does not exist" % im0_path)
    if im2 is None:
        raise Exception("File %s does not exist" % im1_path)


    # Convert im1 and im2 to GRAY.
    im1GRAY = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
    im2GRAY = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)

    # Detect features with:
    fast = cv2.FastFeatureDetector_create(threshold=50, nonmaxSuppression=True) # crear detector de features con fast
    im1KPts = fast.detect(im1GRAY, None)
    #im1KPts = thresholdTrans(im1KPts, 20)

    # Draw keypoints
    im1KPtsOut = im1.copy() #
    cv2.drawKeypoints(im1GRAY, im1KPts, outImage=im1KPtsOut, color=(0, 0, 255))


    im1KPts_means = cv2.KeyPoint_convert(im1KPts) # puntos en formato (x, y) en la imagen 1
    #    plt.scatter(im1KPts_means[:, 0], 1KPts_means[:, 1], marker='+', s=100)
    w1, h1 = im1.shape[1], im1.shape[0]

    im1KPts_means = thresholdRot(im1KPts_means, 40, w1, h1)
    im1KPts_means = thresholdTrans(im1KPts_means, 50, w1, h1)

    # Track the keypoints in other frame
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, 0.001)## 40, 0.001
    im2KPts_means, im2KPts_status, im2KPts_err = cv2.calcOpticalFlowPyrLK(im1, im2, im1KPts_means, None, winSize=(21, 21), maxLevel=3, criteria=criteria, minEigThreshold=0.001)


    # Keep only valid points
    im2KPts_status = im2KPts_status.astype(np.bool).ravel()

    # im2valid = thresholdTrans(im2KPts_means, 20)
    # im2KPts_status = im2valid & im2KPts_status

    im2KPts_ok = im2KPts_means[im2KPts_status]
    im1KPts_ok = im1KPts_means[im2KPts_status]


    colRands = np.random.randint(0,255, (100, 3)) # dibujar match entre puntos img1, img2
    im1KltOut = im1.copy()
    for i, (new, old) in enumerate(zip(im2KPts_ok, im1KPts_ok)):
        a, b = new.ravel()
        c, d = old.ravel()
        im1KltOut = cv2.line(im1KltOut, (a, b), (c, d), (0, 0, 255), 1)
        im1KltOut = cv2.circle(im1KltOut, (a, b), 5, colRands[i%len(colRands)].tolist(), -1)
    # img = cv2.add(frame, mask)

    # Use the calibration matrix and keypoints matching to compute the essential matrix
    camMatrix = np.array([[7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, 4.538225000000e+01],
                          [0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, 1.130887000000e-01],
                          [0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 3.779761000000e-03]])

    fx = camMatrix[0][0]
    fy = camMatrix[1][1]
    focal= fx # TODO: Focal length in pixel
    x = camMatrix[0][2]
    y = camMatrix[1][2]

    pp = [x,y]  # TODO: Set the center point [x, y]

    E, mask = cv2.findEssentialMat(im2KPts_ok, im1KPts_ok, focal = focal, pp=(pp[0], pp[1]), method=cv2.RANSAC, prob=0.999, threshold=1.0)
    points, R, t, mask = cv2.recoverPose(E, im2KPts_ok, im1KPts_ok)
    print("Matrix R = {}".format(R))
    print("Matrix t = {}".format(t))
    t = abs(t)

    # Display the figures
    plt.figure(1)
    plt.clf()
    plt.subplot(3, 1, 1)
    plt.imshow(im1KPtsOut[..., ::-1])
    plt.subplot(3, 1, 2)
    plt.imshow(im1KltOut[..., ::-1])
    plt.subplot(3, 1, 3)
    plt.imshow(im2[..., ::-1])

    # Compute R, t from absolute scale and essential matrix
    scale = odotools.getAbsoluteScale(len(poses)+1)




    if t_p is None:
        t_p = np.zeros_like(t)
        R_p = np.asmatrix(np.identity(3))
        R2 = R_p




    t_p = t_p + scale * (R_p * t)

    #
    # R2 = R_p
    R_p = R * R_p
    #
    # senod = R2[0,1]
    # senod2 = R_p[0,1]

    #print(R_p)
    #print(R2)

    # if np.sign(senod) != np.sign(senod2):
    #     t_p = abs(t_p)
    #     print("hola")

    # Stack up the poses
    poses = np.vstack([poses, [t_p[0, 0], t_p[2, 0]]])  # x, y
    gt_poses = odotools.gt_poses[1:len(poses)+1, 0:2]

    # Compute errors
    # odo_dist = np.sqrt(np.sum(np.diff(poses, axis=0)**2))
    t_error = np.mean(np.sqrt(np.sum((gt_poses - poses)**2, axis=1)))

    # Plot trajectory and GT
    plt.figure(2)
    plt.clf()
    plt.plot(poses[:, 0], poses[:, 1], marker='o', c='b', label="Odometry")
    plt.plot(gt_poses[:, 0], gt_poses[:, 1], marker='s', c='g', label="GroundTruth")
    plt.axis('equal')
    plt.title("Frame %d\nCum. err. %.2fm" % (fIdx, t_error))
    plt.legend()

    plt.draw()
    plt.waitforbuttonpress(0.02)

    fIdx += 1
