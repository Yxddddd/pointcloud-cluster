#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include "dbscan.h"

// using namesapce std;
int DBSCAN::cluster(){
    int clusterID = 1;
    for(int i=0;i<m_points->points.size();i++){
    	if(m_points->points[i].intensity<0){
    		if(expandCluster(m_points->points[i],clusterID)!=FAILURE)
    			clusterID++;
    	}
    }
    return clusterID;
}
std::vector<int> DBSCAN::calculateCluster(pcl::PointXYZI point)
{
    std::vector<int> clusterIndex;

    for(int i=0;i<m_points->points.size();i++){
    	if(calculateDistance(point, m_points->points[i])<= m_epsilon){
    		clusterIndex.push_back(i);
    	}
    }
    return clusterIndex;

}
int DBSCAN::expandCluster(pcl::PointXYZI point, int clusterID)
{    
    std::vector<int> clusterSeeds = calculateCluster(point);

    if ( clusterSeeds.size() < m_minPoints )
    {
        point.intensity = NOISE;
        return FAILURE;
    }
    else
    {
        int index = 0, indexCorePoint = 0;
        std::vector<int>::iterator iterSeeds;
        for( iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
        {
        	m_points->points[*iterSeeds] = clusterID;
            if (m_points->points[*iterSeeds].x == point.x && m_points->points[*iterSeeds].y == point.y && m_points->points[*iterSeeds].z == point.z )
            {
                indexCorePoint = index;
            }
            ++index;
        }
        clusterSeeds.erase(clusterSeeds.begin()+indexCorePoint);

        for(int i=0,n=clusterSeeds.size();i<n;i++){
        	std::vector<int> clusterNeighors = calculateCluster(m_points->points[clusterSeeds[i]]);
        	if(clusterNeighors.size()>=m_minPoints){
        		for(auto iter = clusterNeighors.begin();iter!=clusterNeighors.end();++iter){
        			if(m_points->points[*iter].intensity == UNCLASSIFIED || m_points->points[*iter].intensity == NOISE){
        				if(m_points->points[*iter].intensity == UNCLASSIFIED){
        					clusterSeeds.push_back(*iter);
        					n = clusterSeeds.size();
        				}
        				m_points->points[*iter].intensity = clusterID;
        			}
        		}
        	}
        }

        return SUCCESS;
    }
}
inline double DBSCAN::calculateDistance( pcl::PointXYZI pointCore, pcl::PointXYZI pointTarget )
{
    return pow(pointCore.x - pointTarget.x,2)+pow(pointCore.y - pointTarget.y,2)+pow(pointCore.z - pointTarget.z,2);
}
