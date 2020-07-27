import sys



import tf
import os
import cv2
import rospy
import rosbag
import progressbar
from tf2_msgs.msg import TFMassage
from datetime import datetime
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform
import numpy as np
import argparse
import glob


class raw:
    """Load and parse raw data into a usable format"""

    def __init__(self, dataset_path, sequence_number, **kwargs):
        self.data_path = os.path.join(dataset_path, sequence_number)

        self.frames = kwargs.get('frames', None)

        self.imtype = kwargs.get('imtype', 'png')

        self._get_file_lists()
        #self._load_calib()
        self._load_timestamps()

    def _get_file_lists(self):

        self.cam0_files = sorted(glob.glob(
            os.path.join(self.data_path, 'image_2', '*.{}'.format(self.imtype))))
        
        self.cam1_files = sorted(glob.glob(
            os.path.join(self.data_path, 'image_3', '*.{}'.format(self.imtype))))

        self.velo_files = sorted(glob.glob(
            os.path.join(self.data_path, 'velodyne', '*.bin')))

        # if self.frames is not None:

    def _load_timestamps(self):
        timestamp_file = os.path.join(
                self.data_path, 'times.txt')

        self.timestamps = []
        with open(timestamp_file, 'r') as f:
            for line in f.readlines():
                number = float(line[0:7])
                sign = 1.0

                if line[9]=='+':
                    sign = 1.0
                else:
                    sign = -1.0

                num = float(line[10:11])

                time_t = number*(10**(sign*num))

                self.timestamps.append(time_t)



def run_semantickitti2bag():

    parser = argparse.ArgumentParser(description='Convert SemanticKITTI dataset to rosbag file')

    odometry_sequences = []
    for s in range(22):
        odometry_sequences.append(str(s).zfill(2)) # 1 -> 01

    parser.add_argument('dataset_path', help='Path to Semantickitti file')
    parser.add_argument('sequence_number', help='Sequence number, must be written as 1 to 01')
    args = parser.parse_args()

    bridge = CvBridge()
    compression = rosbag.Compression.NONE

    #camera

    cameras = [
            (0, 'camera_left', '/semantickitti/camera_left'),
            (1, 'camera_right', '/semantickitti/camera_right'/)
        ]
    
    if args.dataset_path == None:
        print("Dataset path is not given.")
        sys.exit(1)
    elif args.sequence_number == None:
        print("Sequence number is not given.")
        sys.exit(1)

    bag = rosbag.Bag("semantickitti_sequence{}_{}.bag".format(args.sequence_number, datetime.datetime.now()), 'w', compression=compression)

    #kitti
