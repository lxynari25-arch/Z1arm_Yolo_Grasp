# import numpy
# x, y = (397, 121)
# data = numpy.load("/home/unitree/lxy_arm/robot_grasp/core/../output/imgSave/debug/raw_depth_20260127_161633_160.npy")
# print(data[240])


import sys
import numpy as np

core_path = "../core"
sys.path.append(core_path)
import signal
import sys
import time
import argparse
import numpy as np
from ultralytics import YOLO
from realsense2 import *
from wrap_z1 import z1_arm
from calibrate import Calibration
from pathlib import Path
from utils import *

model = YOLO("/home/unitree/lxy_arm/ultralytics/yolo26n-objv1-150.pt")
print(model.names)
