
#include "utility.h"
#include "dbscan.h"
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/passthrough.h> 
#include <pcl/filters/statistical_outlier_removal.h>
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

	cv::Mat rangeMat;		
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
        copyPointCloud(laserCloudMsg);
        findStartEndAngle();	
        projectPointCloud();	
        groundRemoval();		
        cloudSegmentation();	
        publishCloud();			
        resetParameters();		
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
        float verticalAngle, horizonAngle, range;		
        size_t rowIdn, columnIdn, index, cloudSize; 	
        pcl::PointXYZI thisPoint;

        cloudSize = laserCloudIn->points.size();

        for (size_t i = 0; i < cloudSize; ++i){
			
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
			
            verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;//计算竖直角度
            rowIdn = (verticalAngle + ang_bottom) / ang_res_y;	
																
            if (rowIdn < 0 || rowIdn >= N_SCAN)					
                continue;
			
            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;	
			
            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;	
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;
			
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;
			
            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            rangeMat.at<float>(rowIdn, columnIdn) = range;
			
            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;
			
            index = columnIdn  + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
            count++;
        }
    }

    void groundRemoval(){
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;

        for (size_t j = 0; j < Horizon_SCAN; ++j){
            for (size_t i = 0; i < groundScanInd; ++i){

                lowerInd = j + ( i )*Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN;

                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1){
                    groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }
                
                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;
                if (abs(angle - sensorMountAngle) <= 15){//sensorMountAngle = 0
                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;//groundMat中，值为1代表为地面点
                }
            }
        }

        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX){
                    labelMat.at<int>(i,j) = -1;
                }
            }
        }
        // if (pubGroundCloud.getNumSubscribers() != 0){
        for (size_t i = 0; i <= groundScanInd; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1)
                    groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
            }
        }
    }

    void cloudSegmentation(){
        for (size_t i = 0; i < N_SCAN; ++i)
            for (size_t j = 0; j < Horizon_SCAN; ++j)
                if (labelMat.at<int>(i,j) == 0)
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
	    //std::cout<<"PointCloudCluster number "<<PointCloudCluster->points.size()<<std::endl;
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
        //std::cout<<"--cost time "<<ros::Time::now().toSec()-time<<"cluster number is "<<cluster_indices.size()<<" points number "<<PointCloudCluster->points.size()<<std::endl;

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
        while(queueSize > 0){
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;//
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;

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
				
                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));

                if ((*iter).first == 0)
                    alpha = segmentAlphaX;
                else
                    alpha = segmentAlphaY;
				
                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));
                if (angle > segmentTheta){

                    queueIndX[queueEndInd] = thisIndX;
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }
        bool feasibleSegment = false;

        if (allPushedIndSize >= 30)
            feasibleSegment = true;

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
        pubGroundCloud.publish(laserCloudTemp);

        pcl::toROSMsg(*PointCloudWithLabel, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "velodyne";
        pubPointCloudWithLabel.publish(laserCloudTemp);


        pcl::toROSMsg(*fullCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "velodyne";
        pubFullCloud.publish(laserCloudTemp);

        pcl::toROSMsg(*PointCloudCluster, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "velodyne";
        pubPointCloudCluster.publish(laserCloudTemp);

        
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
