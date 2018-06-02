import os
import numpy as np


gt_poses = np.zeros((0, 2), dtype=np.float)  # (x, y) groundtruth pose
def read_poses(path, db, fIdx):
    global gt_poses
    """Generator to load ground truth poses (T_w_cam0) from file."""
    pose_file = os.path.join(path, '%s.txt' % db)

    # Read and parse the poses
    try:
        with open(pose_file, 'r') as f:
            lines = f.readlines()[fIdx:]
            print(lines[0])
            T_w_cam0 = np.fromstring(lines[0], dtype=float, sep=' ')
            T_w_cam0 = T_w_cam0.reshape(3, 4)
            print(T_w_cam0)
            T_w_cam0 = np.vstack((T_w_cam0, [0, 0, 0, 1]))
            print(T_w_cam0[0, 3])


            for line in lines:
                T_w_cam0 = np.fromstring(line, dtype=float, sep=' ')
                T_w_cam0 = T_w_cam0.reshape(3, 4)
                T_w_cam0 = np.vstack((T_w_cam0, [0, 0, 0, 1]))
                gt_poses = np.vstack([gt_poses, [T_w_cam0[0, 3], T_w_cam0[2, 3]]])

            gt_poses -= gt_poses[0]
            np.savetxt('GPS_data', gt_poses, fmt='%1.12e')

    except FileNotFoundError:
        print('Ground truth poses are not avaialble for sequence ' +
              pose_file + '.')

def getAbsoluteScale(idx):
    global gt_poses
    return np.sqrt(np.sum((gt_poses[idx]-gt_poses[idx-1])**2))