# semantickitti2bag

"semantickitti2bag" contains helpful python programs to convert SemanticKITTI dataset to rosbag file.

 * Link to original [KITTI Odometry dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)
 * Link to [SemanticKITTI dataset](http://semantic-kitti.org/)

You can convert these dataset easy way :)

![rviz_2020_08_02](https://user-images.githubusercontent.com/60866340/89119958-13a2de80-d4ed-11ea-8ffc-29a5c5f5f420.png)

This repository is based on [tomas789's](https://github.com/tomas789) [kitti2bag](https://github.com/tomas789/kitti2bag) and [PRBonn's](https://github.com/PRBonn) [semantic-kitti-api](https://github.com/PRBonn/semantic-kitti-api). These code are MIT Licence.

# Data organization

Data must be organized in following format.

'''bash
/kitti/dataset/
	└── sequences/
		├── 00/
		│   │   
		│   ├── image_0/
		│   │     ├ 000000.png
		│   │     └ 000001.png
                  │   ├── image_1/
                  │   ├── image_2/
                  │   ├── image_3/
                  │   ├── labels/
                  │   │     ├ 000000.label
                  │   │     └ 000001.label
                  |   ├──poses.txt 
                  |   ├──calib.txt <- From directory of "data_odometry_calib"
                  |   ├──times.txt
                  |   ├──00.txt  <- Ground truth 
                  |   |   
                  |   |   
                  |   | 
                  │   └── velodyne/
                  │         ├ 000000.bin
                  │         └ 000001.bin
                  ├── 01/
                  ├── 02/
                  .
                  .
                  .
                  └── 21/

'''

SemanticKITTI dataset has voxel data, but this repository doesn't handle.

 * image 0~1 is monocolor, and image 2~3 is colored.
 * velodyne contains the pointcloud for each scan.Each .bin is list of float32 points in [x, y, z, intensity] format.
 * .label file cantains a uint32 label for each point in the corresponding .bin scan. upper 16 bit contains instance id and lower 16 bit contains semantic label(ex.. car, bicycle, people). This program doesn't implement instance id.
 * poses.txt contain pose in world coordinate as homogenerous matrix. But this file must be loaded with calib.txt otherwise you can't get correct pose.
 * times.txt contains timestamps for each data(LiDAR, image, pose) scan
 * sequencenumber.txt(ex.. 00.txt) contains ground truth poses. 

# How to install it?

'''bash
cd catkin_ws/src
git clone https://github.com/amslabtech/semantikitti2bag
'''

#How to run it

'''bash
cd catkin_ws/src/semantickitti2bag
python __main__.py -p /place/your/kitti/dataset/ -s 00
'''

 * -s 00 specify sequence number you must write number as 0 to  00, 1 to 01, 11 to 11.

# Author

 * [Hibiki1020](https://github.com/Hibiki1020)
 * Meiji university

# License
This repository is under MIT License.
