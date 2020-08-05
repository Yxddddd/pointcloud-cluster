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