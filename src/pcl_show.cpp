#include "utility.h"
#include <pcl/point_types.h>
#include <fstream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <limits>

using namespace std;

vector<vector<float>>color_rgb{{128,0,0},{255,0,0},{255,0,255},{255,153,204},{153,51,0,},{219,112,147},{255,102,0,},{255,153,0},{255,204,0},
								{255,204,153},{51,51,0},{128,128,0},{153,204,0},{255,255,0},{255,255,153},{0,51,0},{0,128,0},{255,69,0},{255,99,71},
								{51,153,102},{0,255,0},{204,255,204},{0,51,102},{0,128,128},{51,204,204},{0,255,255},{204,255,255},{85,107,47},
								{0,0,128},{0,0,255},{51,102,255},{0,204,255},{153,204,255},{51,51,153},{102,102,153},{128,0,128},{255,165,0},{34,139,34},
								{153,51,102},{204,153,255},{51,51,51},{128,128,128},{153,153,153},{192,192,192},{0,255,127},{255,215,0},{192,14,235}};

template <typename T1, typename T2>
T1 addPointXYZI(T1& a, T2& b){
	T1 c;
	c.x = a.x + b.x;
	c.y = a.y + b.y;
	c.z = a.z + b.z;
	c.intensity = a.intensity;
	return c;
}
template <typename T>
T minusPointXYZI(T& a, T& b){
	T c;
	c.x = a.x - b.x;
	c.y = a.y - b.y;
	c.z = a.z - b.z;
	c.intensity = a.intensity;
	return c;
}
vector<vector<float>>buttomBox{{-0.5,-0.5,-0.5},{-0.5,-0.5,0.5},{0.5,-0.5,0.5},{0.5,-0.5,-0.5}};
vector<vector<float>>topBox{{-0.5,0.5,-0.5},{-0.5,0.5,0.5},{0.5,0.5,0.5},{0.5,0.5,-0.5}};
vector<pcl::PointXYZI>paintLine;
//////////////////////////
pcl::PointXYZI max_point;
pcl::PointXYZI min_point;
pcl::PointXYZI center;
//////////////////////////
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloud_XYZI(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloud_withLabel(new pcl::PointCloud<pcl::PointXYZI>());
vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>label_vec;	//points different from label
pcl::PointCloud<pcl::PointXYZI>::Ptr label_value(new pcl::PointCloud<pcl::PointXYZI>());
unordered_set<int>intensity_set;
unordered_map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>labelPointCloud;
unordered_map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr>labelPointCloudXYZI;
//////////////////////////
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

void visualize_pcd(){
	ros::Rate rate(20);
	while(ros::ok()){
		viewer->removeAllPointClouds(); 
		viewer->removeAllShapes();   
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(PointCloud_RGB);
	    viewer->addPointCloud<pcl::PointXYZRGB>(PointCloud_RGB, rgb, "sample cloud");
	    char str[paintLine.size()/2];
	    for(int i=0;i<paintLine.size();){
			sprintf(str, "%d", i);
			viewer->addLine<pcl::PointXYZI>(paintLine[i],paintLine[i+1],255,255,0,str);
			i = i+2;
		}
		std::cout<<"point cloud rgb "<<PointCloud_RGB->points.size()<<" line points "<<paintLine.size()<<std::endl;
		viewer->spinOnce(10);
		// boost::this_thread::sleep (boost::posix_time::microseconds (100));
	}
}
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn){
	double time = ros::Time::now().toSec();
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::fromROSMsg(*laserCloudIn,*laserCloud);
	int cloudSize = laserCloud->points.size();

	PointCloud_withLabel->clear();
	labelPointCloud.clear();
	paintLine.clear();
	PointCloud_RGB->clear();

	for(int i=0;i<cloudSize;++i){
		pcl::PointXYZI thisPoint = laserCloud->points[i];
		if(!intensity_set.count(thisPoint.intensity) && thisPoint.intensity >=0)
			intensity_set.insert(thisPoint.intensity);
		if(thisPoint.intensity >=0){
			PointCloud_withLabel->push_back(thisPoint);
		}
	}
	int cloudLabelSize = PointCloud_withLabel->points.size();
	for(int i=0;i<cloudLabelSize;i++){
		pcl::PointXYZRGB thisPoint;
		thisPoint.x = PointCloud_withLabel->points[i].x;
		thisPoint.y = PointCloud_withLabel->points[i].y;
		thisPoint.z = PointCloud_withLabel->points[i].z;
		// thisPoint.intensity = PointCloud_withLabel->points[i].intensity;
		int intensity = PointCloud_withLabel->points[i].intensity;
		thisPoint.r = color_rgb[intensity%color_rgb.size()][0];
		thisPoint.g = color_rgb[intensity%color_rgb.size()][1];
		thisPoint.b = color_rgb[intensity%color_rgb.size()][2];;
		PointCloud_RGB->push_back(thisPoint);

		auto iter = labelPointCloud.find(intensity);
		if(iter == labelPointCloud.end()){
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>());
			labelPointCloud[intensity]=tmp;
		}
		labelPointCloud[intensity]->push_back(thisPoint);
	}
	for(int i=0;i<labelPointCloud.size();i++){
		for(int j=0;j<labelPointCloud[i]->points.size();++j){
			pcl::PointXYZRGB thisPoint = labelPointCloud[i]->points[j];
			center = addPointXYZI(center, thisPoint);
			max_point.x = max(max_point.x, thisPoint.x);
			max_point.y = max(max_point.y, thisPoint.y);
			max_point.z = max(max_point.z, thisPoint.z);

			min_point.x = min(min_point.x, thisPoint.x);
			min_point.y = min(min_point.y, thisPoint.y);
			min_point.z = min(min_point.z, thisPoint.z);
			if(j == labelPointCloud[i]->points.size()-1){
				center.x = center.x/(labelPointCloud[i]->points.size()*1.0);
				center.y = center.y/(labelPointCloud[i]->points.size()*1.0);
				center.z = center.z/(labelPointCloud[i]->points.size()*1.0);
				float scale_x = (max_point.x - min_point.x)>0?(max_point.x - min_point.x):0;
				float scale_y = (max_point.y - min_point.y)>0?(max_point.y - min_point.y):0;
				float scale_z = (max_point.z - min_point.z)>0?(max_point.z - min_point.z):0;

				for(int i=0;i<4;i++){
					if(i==3){
						pcl::PointXYZI first_point;
						first_point.x = center.x + buttomBox[i][0]*scale_x;
						first_point.y = center.y + buttomBox[i][1]*scale_y;
						first_point.z = center.z + buttomBox[i][2]*scale_z;
						paintLine.push_back(first_point);
						pcl::PointXYZI second_point;
						second_point.x = center.x + buttomBox[0][0]*scale_x;
						second_point.y = center.y + buttomBox[0][1]*scale_y;
						second_point.z = center.z + buttomBox[0][2]*scale_z;
						paintLine.push_back(second_point);
					}else{
						pcl::PointXYZI first_point;
						first_point.x = center.x + buttomBox[i][0]*scale_x;
						first_point.y = center.y + buttomBox[i][1]*scale_y;
						first_point.z = center.z + buttomBox[i][2]*scale_z;
						paintLine.push_back(first_point);
						pcl::PointXYZI second_point;
						second_point.x = center.x + buttomBox[i+1][0]*scale_x;
						second_point.y = center.y + buttomBox[i+1][1]*scale_y;
						second_point.z = center.z + buttomBox[i+1][2]*scale_z;
						paintLine.push_back(second_point);
					}
				}
				for(int i=0;i<4;i++){
					if(i==3){
						pcl::PointXYZI first_point;
						first_point.x = center.x + topBox[i][0]*scale_x;
						first_point.y = center.y + topBox[i][1]*scale_y;
						first_point.z = center.z + topBox[i][2]*scale_z;
						paintLine.push_back(first_point);
						pcl::PointXYZI second_point;
						second_point.x = center.x + topBox[0][0]*scale_x;
						second_point.y = center.y + topBox[0][1]*scale_y;
						second_point.z = center.z + topBox[0][2]*scale_z;
						paintLine.push_back(second_point);
					}else{
						pcl::PointXYZI first_point;
						first_point.x = center.x + topBox[i][0]*scale_x;
						first_point.y = center.y + topBox[i][1]*scale_y;
						first_point.z = center.z + topBox[i][2]*scale_z;
						paintLine.push_back(first_point);
						pcl::PointXYZI second_point;
						second_point.x = center.x + topBox[i+1][0]*scale_x;
						second_point.y = center.y + topBox[i+1][1]*scale_y;
						second_point.z = center.z + topBox[i+1][2]*scale_z;
						paintLine.push_back(second_point);
					}
				}
				for(int i=0;i<4;i++){

					pcl::PointXYZI first_point;
					first_point.x = center.x + topBox[i][0]*scale_x;
					first_point.y = center.y + topBox[i][1]*scale_y;
					first_point.z = center.z + topBox[i][2]*scale_z;
					paintLine.push_back(first_point);
					pcl::PointXYZI second_point;
					second_point.x = center.x + buttomBox[i][0]*scale_x;
					second_point.y = center.y + buttomBox[i][1]*scale_y;
					second_point.z = center.z + buttomBox[i][2]*scale_z;
					paintLine.push_back(second_point);	
				}
				max_point.x = std::numeric_limits<float>::lowest();		
				max_point.y = std::numeric_limits<float>::lowest();
				max_point.z = std::numeric_limits<float>::lowest();

				min_point.x = std::numeric_limits<float>::max();
				min_point.y = std::numeric_limits<float>::max();
				min_point.z = std::numeric_limits<float>::max();
				
				center.x = 0;	center.y = 0;	center.z = 0;
			}
		}
	}
	std::cout<<"cost time is ==> "<<ros::Time::now().toSec()-time<<std::endl;
}
int main(int argc, char**argv){

	ros::init(argc,argv,"pcl_show");
	ros::NodeHandle nh;
	ros::Subscriber sunLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/pointcloud_cluster",2,laserCloudHandler);
	string str = "/home/yxd/catkin_ws/src/lidar_preprocess/data/point1_64.txt";
	viewer->setCameraPosition(
        0, 0, 100,                                // camera位置
        0, 0, 0,                                // view向量--相机朝向
        0, 0, 0                                 // up向量
        );

	vector<pcl::PointXYZI>PointMsgs;
	
	
	center.x = 0;	center.y = 0;	center.z = 0;
	
	max_point.x = std::numeric_limits<float>::lowest();		
	max_point.y = std::numeric_limits<float>::lowest();
	max_point.z = std::numeric_limits<float>::lowest();

	
	min_point.x = std::numeric_limits<float>::max();
	min_point.y = std::numeric_limits<float>::max();
	min_point.z = std::numeric_limits<float>::max();
	// viewer->reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	boost::thread visualizethread(visualize_pcd);
	ros::Rate rate(30);
	bool status = ros::ok();
	while(status){
		ros::spinOnce();
		status = ros::ok();
		rate.sleep();
	}
	visualizethread.join();

	return 0;
}