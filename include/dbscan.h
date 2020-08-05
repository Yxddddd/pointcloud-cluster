#ifndef DBSCAN_H
#define DBSCAN_H
#include <iostream>
#include <vector>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

class DBSCAN{
public:
	DBSCAN(unsigned int minPts, float eps, pcl::PointCloud<pcl::PointXYZI>::Ptr points){
        m_minPoints = minPts;
        m_epsilon = eps;
        m_points = points;
        m_pointSize = points->points.size();
    }
    ~DBSCAN(){}
	int cluster();
	int expandCluster(pcl::PointXYZI point, int clusterID);
	std::vector<int> calculateCluster(pcl::PointXYZI point);
	inline double calculateDistance(pcl::PointXYZI pointCore, pcl::PointXYZI pointTarget);
	int getTotalPointSize() {return m_pointSize;}
    int getMinimumClusterSize() {return m_minPoints;}
    int getEpsilonSize() {return m_epsilon;}
    pcl::PointCloud<pcl::PointXYZI>::Ptr getCluster(){return m_points;}
private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_points;
    unsigned int m_pointSize;	//points number
    unsigned int m_minPoints;
    float m_epsilon;
};

#endif