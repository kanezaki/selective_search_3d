# 3D Selective Search

This code produces object candidates (in bounding boxes) from RGBD frames.

If you use this software, please cite the following paper: 

Asako Kanezaki and Tatsuya Harada. 
**3D Selective Search for Obtaining Object Candidates.** 
*IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2015.   
([pdf](http://www.mi.t.u-tokyo.ac.jp/kanezaki/pdf/IROS2015_kanezaki.pdf))

## Requirement

ROS indigo/jade  
http://wiki.ros.org/

## Usage

### 1. Launch Kinect.  
For example,  
  `$ roslaunch freenect_launch freenect.launch`  
In this case, you will get point cloud in rostopic */camera/depth_registered/points* and color images in rostopic */camera/rgb/image_color*.  

### 2. Run 3D selective search.  
  `$ rosrun selective_search_3d selective_search_3d -v 0.02 -d 1.0 points:=/camera/depth_registered/points`  
In this case, you will get bounding boxes of object candidates within depth range = 1.0m. The voxel resolution is set to 0.02m.

### 3. Show bounding boxes. (2D)  
  `$ rosrun selective_search_3d show_bbox -n 5 image:=/camera/rgb/image_color`  
or  
  `$ python show_bbox.py -n 5 image_topic:=/camera/rgb/image_color`  
In this case, five bounding boxes at maximum are shown.  

### 4. Print bounding boxes. (3D)  
  `$ rosrun selective_search_3d print_bbox3D -n 5`  
or  
  `$ python print_bbox3D.py -n 5`  
In this case, five bounding boxes at maximum are printed.  

### 5. Run 3D selective search offline.  
  `$ rosrun selective_search_3d selective_search_3d_offline -v 0.02 -p cloud.pcd -o bbox.txt`  
In this case, you will get bounding boxes of object candidates written into bbox.txt. The voxel resolution is set to 0.02m.  

## Licence

BSD

