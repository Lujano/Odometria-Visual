import cv2
import yaml
import numpy as np

import cv2
fs = cv2.FileStorage("Output_fast.yaml", cv2.FILE_STORAGE_READ)
fn = fs.getNode("Trayectoria")
data = np.asanyarray(fn.mat())
print(fn.mat())
print(data[0])