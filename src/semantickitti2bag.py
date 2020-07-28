import sys
sys.dont_write_bytecode = True

import utils #import utils.py

import tf
import os
import cv2
from cv_bridge import CvBridge
import rospy
import rosbag
import progressbar
from tf2_msgs.msg import TFMessage
import datetime
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform
import numpy as np
import argparse
import glob


class SemanticKitti_Raw:
    """Load and parse raw data into a usable format"""

    def __init__(self, dataset_path, sequence_number, scanlabel_bool, **kwargs):
        self.data_path = os.path.join(dataset_path, 'dataset', 'sequences', sequence_number)

        self.frames = kwargs.get('frames', None)

        self.imtype = kwargs.get('imtype', 'png')

        self._get_file_lists(scanlabel_bool)
        #self._load_calib()
        
        self._load_timestamps()

    def _get_file_lists(self, scanlabel_bool):

        self.cam0_files = sorted(glob.glob(
            os.path.join(self.data_path, 'image_2', '*.{}'.format(self.imtype))))
        
        self.cam1_files = sorted(glob.glob(
            os.path.join(self.data_path, 'image_3', '*.{}'.format(self.imtype))))

        self.velo_files = sorted(glob.glob(
            os.path.join(self.data_path, 'velodyne', '*.bin')))

        if scanlabel_bool == 1:
            self.label_files = sorted(glob.glob(
                os.path.join(self.data_path, 'labels', '*.label')))
        #print(self.cam1_files)
        #print(self.velo_files)

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

                num = float(line[10])*10 + float(line[11])*1

                time_t = number*(10**(sign*num))
                #print(time_t)
                self.timestamps.append(time_t)

def save_velo_data_with_label(bag, kitti, velo_frame_id, velo_topic):
    print("Exporting Velodyne and Label data")
    
    velo_data_dir = os.path.join(kitti.data_path, 'velodyne')
    velo_filenames = sorted(os.listdir(velo_data_dir))

    label_data_dir = os.path.join(kitti.data_path, 'labels')
    label_filenames = sorted(os.listdir(label_data_dir))

    datatimes = kitti.timestamps

    iterable = zip(datatimes, velo_filenames, label_filenames)
    bar = progressbar.ProgressBar()

    for dt, veloname, labelname in bar(iterable):
        if dt is None:
            continue

        velo_filename = os.path.join(velo_data_dir, veloname)
        label_filename = os.path.join(label_data_dir, labelname)

        veloscan = (np.fromfile(velo_filename, dtype=np.float32)).reshape(-1, 4)
        labelscan = (np.fromfile(label_filename, dtype=np.int32)).reshape(-1,1)
        
        labeldata = utils.LabelDataConverter(labelscan)
        
        scan = []

        for t in range(len(labeldata.rgb_id)):
            point = [veloscan[t][0], veloscan[t][1], veloscan[t][2], veloscan[t][3], labeldata.rgb_id[t]]
            scan.append(point)

        header = Header()
        header.frame_id = velo_frame_id
        header.stamp = rospy.Time.from_sec(float(dt))

        fields =[PointField('x',  0, PointField.FLOAT32, 1),
                 PointField('y',  4, PointField.FLOAT32, 1),
                 PointField('z',  8, PointField.FLOAT32, 1),
                 PointField('i', 12, PointField.FLOAT32, 1),
                 PointField('rgb', 16, PointField.UINT32, 1)]

        pcl_msg = pcl2.create_cloud(header, fields, scan)
        bag.write(velo_topic + '/pointcloud', pcl_msg, t=pcl_msg.header.stamp)

def run_semantickitti2bag():

    parser = argparse.ArgumentParser(description='Convert SemanticKITTI dataset to rosbag file')


    parser.add_argument("-p","--dataset_path", help='Path to Semantickitti file')
    parser.add_argument("-s","--sequence_number", help='Sequence number, must be written as 1 to 01')
    args = parser.parse_args()

    bridge = CvBridge()
    compression = rosbag.Compression.NONE

    #camera

    cameras = [
            (0, 'camera_left', '/semantickitti/camera_left'),
            (1, 'camera_right', '/semantickitti/camera_right')
        ]
    
    if args.dataset_path == None:
        print("Dataset path is not given.")
        sys.exit(1)
    elif args.sequence_number == None:
        print("Sequence number is not given.")
        sys.exit(1)

    scanlabel_bool = 1
    if int(args.sequence_number) > 10:
        scanlabel_bool = 0
        
    bag = rosbag.Bag("semantickitti_sequence{}.bag".format(args.sequence_number), 'w', compression=compression)

    kitti = SemanticKitti_Raw(args.dataset_path, args.sequence_number, scanlabel_bool)

    if not os.path.exists(kitti.data_path):
        print('Path {} does not exists. Force-quiting....'.format(kitti.data_path))
        sys.exit(1)

    if len(kitti.timestamps) == 0:
        print('Dataset is empty? Check your semantickitti dataset file')
        sys.exit(1)
    
    try:
        velo_frame_id = 'velo_link'
        velo_topic = '/kitti/velo'

        #tf_static
        #transforms = []

        #util = read_calib_file(os.path.join(kitti.data_path, 'calib.txt'))

        #save_static_transform()
        #save_dynamic_tf(bag, kitti, initial_time=None)
        if scanlabel_bool == 1:
            save_velo_data_with_label(bag, kitti, velo_frame_id, velo_topic)
        elif scanlabel_bool == 0:
            save_velo_data(bag, kitti, velo_frame_id, velo_topic)
        #save_poses(bag, kitti, 
    
    finally:
        print('Convertion is done')
        print(bag)
        bag.close()
