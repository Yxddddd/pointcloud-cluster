
#include "utility.h"
#include "dbscan.h"
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>//聚类

#include <pcl/filters/passthrough.h> //直通滤波器
#include <pcl/filters/statistical_outlier_removal.h>//统计滤波
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>

#include "DBSCAN_simple.h"
#include "DBSCAN_precomp.h"
#include "DBSCAN_kdtree.h"

using namespace cv;

//N_SCAN 16
//Horizon_SCAN 1800
/*
1） groundMat.at<int8_t>(i,j) = 0，初始值；
2） groundMat.at<int8_t>(i,j) = 1，有效的地面点；
3） groundMat.at<int8_t>(i,j) = -1，无效地面点；

1） rangeMat.at(i,j) = FLT_MAX，浮点数的最大值，初始化信息；
2） rangeMat.at(rowIdn, columnIdn) = range，保存图像深度

1） labelMat.at(i,j) = 0，初始值；
2） labelMat.at(i,j) = -1，无效点；
3）labelMat.at(thisIndX, thisIndY) = labelCount，平面点；
4）labelMat.at(allPushedIndX[i], allPushedIndY[i]) = 999999，需要舍弃的点，数量不到30

*/
// const int imuQueLength = 100;   //imu频率
#define MINIMUM_POINTS 4     // minimum number of cluster
#define EPSILON (5.0*5.0)  // distance for clustering, metre^2

class FilterProcess{
private:
	ros::NodeHandle nh;
	ros::Subscriber subLaserCloud;
	ros::Publisher pubFilterPointCloud;
	ros::Publisher pubFullCloud;
	ros::Publisher pubGroundCloud;
	ros::Publisher pubPointCloudWithLabel;
	ros::Publisher pubPointCloudCluster;
	//
	pcl::PointCloud<pcl::PointXYZI>::Ptr fullCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr passCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr outlierCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn;		//laser cloud fron topic
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudPub;
	pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud;		//ground cloud extract from laserCloudIn
	pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudWithLabel;	//intensity label;
	pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudCluster;
	pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudClusterOutPut;

	cv::Mat rangeMat;		//按照深度投影后存储矩阵
    cv::Mat labelMat;
    cv::Mat groundMat;
    int labelCount;

    // float startOrientation;
    // float endOrientation;
    pcl::PointXYZI nanPoint;

    std_msgs::Header cloudHeader;

    std::vector<std::pair<uint8_t, uint8_t> > neighborIterator;

    uint16_t *allPushedIndX;
    uint16_t *allPushedIndY;

    uint16_t *queueIndX;
    uint16_t *queueIndY;
    float startOrientation,endOrientation,orientationDiff;
    int count = 0;
    //////////////////////////////
	int min_cluster_size_param = 20;
	int core_point_min_pts_param = 15;
    float split_theta = 0.0;	//threshold
    float segmentTheta;
    bool first_lidar = true;
    double clusterTolerance=0.5;
    bool save_file = false;
    /////////////////////////////
public:
	FilterProcess():nh("~"){
		nh.getParam("split_theta",split_theta);
		nh.getParam("cluster_tolerance",clusterTolerance);
		nh.getParam("save_file",save_file);
		nh.getParam("min_cluster",min_cluster_size_param);
		nh.getParam("core_min_points",core_point_min_pts_param);
		segmentTheta = split_theta/180.0*M_PI;
		std::cout<<"cluster_tolerance "<<clusterTolerance<<std::endl;
		// std::cout<<"split theta "<<segmentTheta<<std::endl;
		subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(lidarTopic,1,&FilterProcess::cloudHandler,this);
		// subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/pointcloud",1,&FilterProcess::cloudHandler,this);

		pubFullCloud = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud",1);
		pubFilterPointCloud = nh.advertise<sensor_msgs::PointCloud2>("/filter_cloud",1);
		pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_pointcloud",1);
		pubPointCloudWithLabel = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_label",1);
		pubPointCloudCluster = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_cluster",1);

		nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

		allocateMemory();
        resetParameters();
	}
	void allocateMemory(){
		laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());
		laserCloudPub.reset(new pcl::PointCloud<pcl::PointXYZI>());
		passCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
		outlierCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
		groundCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
		PointCloudWithLabel.reset(new pcl::PointCloud<pcl::PointXYZI>());
		PointCloudCluster.reset(new pcl::PointCloud<pcl::PointXYZI>());
		PointCloudClusterOutPut.reset(new pcl::PointCloud<pcl::PointXYZI>());

		fullCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
		fullCloud->points.resize(N_SCAN*Horizon_SCAN);

		std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];
	}
    void resetParameters(){
        laserCloudIn->clear();
        groundCloud->clear();
        outlierCloud->clear();
        PointCloudWithLabel->clear();
        PointCloudCluster->clear();
        PointCloudClusterOutPut->clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(0));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;
        count = 0;

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);

    }
    ~FilterProcess(){}
    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        cloudHeader = laserCloudMsg->header;
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        
    }
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
    	double time = ros::Time::now().toSec();
        copyPointCloud(laserCloudMsg);//点云复制，将rosmsg转换为pcl点云
        findStartEndAngle();	//寻找起始点与最末点的角度转换
        projectPointCloud();	//点云投影
        groundRemoval();		//地面检测
        cloudSegmentation();	//点云分割
        publishCloud();			//发布点云
        resetParameters();		//参数设置
    }
    void findStartEndAngle(){
    	startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
    	endOrientation = -atan2(laserCloudIn->points[laserCloudIn->points.size()-1].y, laserCloudIn->points[laserCloudIn->points.size()-1].x)+2*M_PI;

    	if(endOrientation - startOrientation >3*M_PI){
    		endOrientation -= 2*M_PI;
    	}else if(endOrientation - startOrientation <M_PI)
    		endOrientation += 2*M_PI;
    	orientationDiff = endOrientation - startOrientation;
    }
    void projectPointCloud(){
        float verticalAngle, horizonAngle, range;		//垂直角度，水平角度，
        size_t rowIdn, columnIdn, index, cloudSize; 	//行索引，列索引
        pcl::PointXYZI thisPoint;

        cloudSize = laserCloudIn->points.size();
		//对于激光点云中的每一个点
        for (size_t i = 0; i < cloudSize; ++i){
			
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
			
            verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;//计算竖直角度
            rowIdn = (verticalAngle + ang_bottom) / ang_res_y;	//ang_bottom竖直方向上的起始角度与水平放向的差值ang_bottom = 15.1,ang_res_y=2
																//获得竖直方向上的索引值
            if (rowIdn < 0 || rowIdn >= N_SCAN)					//将rowIdn转换为0-15，最小为(-15 +15.1)/2=0; 最大(15+15.1)/2 = 15, N_SCAN = 16
                continue;
			
            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;	//水平方向上的角度值
			
            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;	//ang_res_x = 0.2,Horizon_SCAN = 1800  获得水平方向上的索引值
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;
			
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;
			
            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);//获得该点的深度
            rangeMat.at<float>(rowIdn, columnIdn) = range;//将该点存入到矩阵中
			
            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;//将索引值保存在该点的强度上，整数部分为垂直方向上索引值，小数部分为水平方向索引值
			
            index = columnIdn  + rowIdn * Horizon_SCAN;//整体上的索引值
            fullCloud->points[index] = thisPoint;//将点云存入到fullCloud中
            count++;
        }
    }
    //地面提取
    void groundRemoval(){
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;

        for (size_t j = 0; j < Horizon_SCAN; ++j){//1800,每个点
            for (size_t i = 0; i < groundScanInd; ++i){//groundScanInd = 7，每条线

                lowerInd = j + ( i )*Horizon_SCAN;//当前线
                upperInd = j + (i+1)*Horizon_SCAN;//下一条激光线对应的索引

                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1){
                    groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }
                
                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;//求两点之间竖直方向上的角度差
				//若相邻两条线之间的俯仰角在10°以内则判断(i,j)为地面点
                if (abs(angle - sensorMountAngle) <= 15){//sensorMountAngle = 0
                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;//groundMat中，值为1代表为地面点
                }
            }
        }

        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX){//rangeMat.at<float>(i,j) == FLT_MAX表示的应该是无效点云，FLT_MAX为极大值
                    labelMat.at<int>(i,j) = -1;//ground point and null point label -1
                }
            }
        }
        // if (pubGroundCloud.getNumSubscribers() != 0){
        for (size_t i = 0; i <= groundScanInd; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1)
                    groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);//如果有节点订阅groundCloud，则将地面点存入groundCloud中
            }
        }
    }
	//点云分割
	//地面点标记值为-1
	//labelMat中，0未分类过的点，-1 地面点，其它为分割后对应的点
    void cloudSegmentation(){
        for (size_t i = 0; i < N_SCAN; ++i)
            for (size_t j = 0; j < Horizon_SCAN; ++j)
                if (labelMat.at<int>(i,j) == 0)//0为没有进行过分类，则通过labelComponents对点云进行分类
                    labelComponents(i, j);

        for(size_t i = 0 ;i<N_SCAN;++i){
        	for(size_t j=0;j<Horizon_SCAN;++j){
        		if(labelMat.at<int>(i,j)>0 && labelMat.at<int>(i,j)!=999999){
        			pcl::PointXYZI thisPoint;
        			thisPoint.x = fullCloud->points[j+i*Horizon_SCAN].x;
        			thisPoint.y = fullCloud->points[j+i*Horizon_SCAN].y;
        			thisPoint.z = fullCloud->points[j+i*Horizon_SCAN].z;
        			thisPoint.intensity = labelMat.at<int>(i,j);
        			PointCloudWithLabel->push_back(thisPoint);
        			thisPoint.intensity = -1;
        			PointCloudCluster->push_back(thisPoint);
        		}
        	}
        }
        double time = ros::Time::now().toSec();
        // std::cout<<"before down size the point cloud number is ==> "<<PointCloudCluster->points.size()<<std::endl;
        if(N_SCAN == 64 || N_SCAN == 32){
        	pcl::VoxelGrid<pcl::PointXYZI>downSizeFilter;
        	downSizeFilter.setLeafSize(0.2,0.2,0.2);
        	downSizeFilter.setInputCloud(PointCloudCluster);
        	downSizeFilter.filter(*PointCloudCluster);
        }
        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
        // std::cout<<"after down size the point cloud number is ==> "<<PointCloudCluster->points.size()<<std::endl;
    	tree->setInputCloud(PointCloudCluster);
    	std::vector<pcl::PointIndices> cluster_indices;
    	DBSCANKdtreeCluster<pcl::PointXYZI> ec;
    	ec.setCorePointMinPts(core_point_min_pts_param);
    	ec.setClusterTolerance(clusterTolerance);
	    ec.setMinClusterSize(min_cluster_size_param);
	    ec.setMaxClusterSize(25000);
	    ec.setSearchMethod(tree);
	    ec.setInputCloud(PointCloudCluster);
	    ec.extract(cluster_indices);
	    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZI>);
	    int label = 0;
	    std::cout<<"PointCloudCluster number "<<PointCloudCluster->points.size()<<std::endl;
	    PointCloudClusterOutPut->clear();
	    for(auto iter = cluster_indices.begin();iter!=cluster_indices.end();++iter, ++label){
	    	for(auto index = iter->indices.begin(); index!= iter->indices.end(); ++index){
	    		pcl::PointXYZI tmp_point;
	    		tmp_point.x = PointCloudCluster->points[*index].x;
	    		tmp_point.y = PointCloudCluster->points[*index].y;
	    		tmp_point.z = PointCloudCluster->points[*index].z;
	    		tmp_point.intensity = label;
	    		PointCloudCluster->points[*index] = tmp_point;
	    	}
	    }
        std::cout<<"--cost time "<<ros::Time::now().toSec()-time<<"cluster number is "<<cluster_indices.size()<<" points number "<<PointCloudCluster->points.size()<<std::endl;
        // if(first_lidar && save_file){
        // 	first_lidar = false;
        // 	write_pointcloud_to_file_XYZI("/home/yxd/catkin_ws/src/lidar_preprocess/data/point1_64.txt",PointCloudCluster);
        // }

    }
    void labelComponents(int row, int col){
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY; 
        bool lineCountFlag[N_SCAN] = {false};

        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;
        //queueSize未处理点的数目
        while(queueSize > 0){
            fromIndX = queueIndX[queueStartInd];//
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;//后移
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;//标记类别

            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){

                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;

                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;

                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;

                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;
				
				//寻找最大的
                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));
				// alpha代表角度分辨率，
				// X方向上角度分辨率是segmentAlphaX(rad)
				// Y方向上角度分辨率是segmentAlphaY(rad)
                if ((*iter).first == 0)//水平方向上的邻点
                    alpha = segmentAlphaX;//水平方向上的分辨率
                else
                    alpha = segmentAlphaY;
				
				// 通过下面的公式计算这两点之间是否有平面特征
				// atan2(y,x)的值越大，d1，d2之间的差距越小,越平坦
                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));
				//如果夹角大于60°，则将这个邻点纳入到局部特征中，该邻点可以用来配准使用
                if (angle > segmentTheta){//1.0472（π/3）

                    queueIndX[queueEndInd] = thisIndX;//将当前点加入队列中
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;//角度大于60°，为同一个标签
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }
        bool feasibleSegment = false;
		//当邻点数目达到30后，则该帧雷达点云的几何特征配置成功
        if (allPushedIndSize >= 30)
            feasibleSegment = true;
		//如果聚类点数小于30大于等于5，统计竖直方向上的聚类点数，若竖直方向上被统计的数目大于3，为有效聚类
        else if (allPushedIndSize >= segmentValidPointNum){//segmentValidPointNum = 5
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= segmentValidLineNum)
                feasibleSegment = true;            
        }

        if (feasibleSegment == true){
            ++labelCount;
        }else{
            for (size_t i = 0; i < allPushedIndSize; ++i){
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
		}

	}
    void publishCloud(){
        sensor_msgs::PointCloud2 laserCloudTemp;

        pcl::toROSMsg(*groundCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "velodyne";
        pubGroundCloud.publish(laserCloudTemp);//发布地面点

        pcl::toROSMsg(*PointCloudWithLabel, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "velodyne";
        pubPointCloudWithLabel.publish(laserCloudTemp);


        pcl::toROSMsg(*fullCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "velodyne";
        pubFullCloud.publish(laserCloudTemp);//发布投影后的全局点云

        pcl::toROSMsg(*PointCloudCluster, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "velodyne";
        pubPointCloudCluster.publish(laserCloudTemp);//发布投影后的全局点云

        
    }
    bool write_pointcloud_to_file_XYZI(string str,pcl::PointCloud<PointType>::Ptr cloudIn){
		std::ofstream of(str,std::ios::app);
		if(!of.is_open()){
			std::cout<<"open file "<<str<<" error!"<<std::endl;
			return false;
		}
		of<<cloudIn->points.size()<<std::endl;
		for(int i=0;i<cloudIn->points.size();++i){
			PointType *pointTmp;
			pointTmp = &cloudIn->points[i];
			of<<pointTmp->x<<" "<<pointTmp->y<<" "<<pointTmp->z<<" "<<pointTmp->intensity<<std::endl;
		}
		of.close();
		return true;
	}
};
int main(int argc, char** argv){

    ros::init(argc, argv, "filterprocess");
    
    FilterProcess FP;

    ROS_INFO("\033[1;32m---->\033[0m intersection recongnition Started.");

    ros::spin();
    return 0;
}
