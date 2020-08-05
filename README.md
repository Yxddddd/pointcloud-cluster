# pointcloud cluster
![image](https://github.com/Yxddddd/pointcloud-cluster/blob/master/imgs/result.PNG)
 
## Dependency  
**ROS** (tested with kinetic)
**PCL**

## How to build with catkin    
```
cd ~/catkin_ws/src  
git clone https://github.com/Yxddddd/pointcloud-cluster.git
cd ~/catkin_ws  
catkin_make 
```
## Running
`roslaunch lidar_gnss_mapping lidar_gnss_mapping.launch`
 
`rosrun lidar_preprocess lidar_preprocess_show`

`rosbag play test.bag`

```
 //HDL-64
extern const string lidarTopic = "/kitti/velo/pointcloud";
extern const int N_SCAN = 64;
extern const int Horizon_SCAN = 2250;
extern const float ang_res_x = 0.16;
extern const float ang_res_y = 0.427;
extern const float ang_bottom = 24.9;
extern const int groundScanInd = 50; 
```
	