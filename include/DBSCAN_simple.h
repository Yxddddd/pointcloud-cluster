#ifndef DBSCAN_SIMPLE_H
#define DBSCAN_SIMPLE_H

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "utility.h"
#include <ctime>

#define UN_PROCESSED 0
#define PROCESSING 1
#define PROCESSED 2

inline bool comparePointClusters (const pcl::PointIndices &a, const pcl::PointIndices &b) {
    return (a.indices.size () < b.indices.size ());
}

template <typename PointT>
class DBSCANSimpleCluster {
public:
    DBSCANSimpleCluster(){
        srand((unsigned int)time(0));
    }
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;
    typedef typename pcl::KdTreeFLANN<PointT>::Ptr KdFLANNPtr;
    virtual void setInputCloud(PointCloudPtr cloud) {
        input_cloud_ = cloud;
    }

    void setSearchMethod(KdFLANNPtr tree) {
        search_method_ = tree;
    }

    void extract(std::vector<pcl::PointIndices>& cluster_indices) {
        std::vector<int> nn_indices;        //index
        std::vector<float> nn_distances;    //distance
        std::vector<bool> is_noise(input_cloud_->points.size(), false);
        std::vector<int> types(input_cloud_->points.size(), UN_PROCESSED);
        for (int i = 0; i < input_cloud_->points.size(); i++) {
            if (types[i] == PROCESSED || is_noise[i]) {	//已经聚类过就不在聚类
                continue;
            }
            //Judging whether it can be the core point according to the number of points
            pcl::PointXYZI thisPoint = input_cloud_->points[i];
            double depth = sqrt(thisPoint.x*thisPoint.x + thisPoint.y*thisPoint.y + thisPoint.z*thisPoint.z);
            double threshold = max((depth*sin(ang_res_y/2.0/180.0*M_PI)*2),eps_);

            search_method_->radiusSearch(input_cloud_->points[i],threshold,nn_indices,nn_distances); 
            int nn_size = nn_indices.size();

            // int nn_size = radiusSearch(i, threshold, nn_indices, nn_distances);  //kdsearch      
            if (nn_size < minPts_) { 
                is_noise[i] = true;
                continue;
            }
            
            std::vector<int> seed_queue;
            seed_queue.push_back(i);
            types[i] = PROCESSED;
            
            for (int j = 0; j < nn_size; j++) {
                if (nn_indices[j] != i) {
                    seed_queue.push_back(nn_indices[j]);
                    types[nn_indices[j]] = PROCESSING;
                }
            } // for every point near the chosen core point.
            int sq_idx = 1;
            while (sq_idx < seed_queue.size()) {
                int cloud_index = seed_queue[sq_idx];
                if (is_noise[cloud_index] || types[cloud_index] == PROCESSED) {//if this point is noise or PROCESSED no need to check neighbors
                    // seed_queue.push_back(cloud_index);
                    types[cloud_index] = PROCESSED;
                    sq_idx++;
                    continue; // no need to check neighbors.
                }
                // nn_size = radiusSearch(cloud_index, eps_, nn_indices, nn_distances);
                pcl::PointXYZI thisPoint = input_cloud_->points[cloud_index];
                double depth = sqrt(thisPoint.x*thisPoint.x + thisPoint.y*thisPoint.y + thisPoint.z*thisPoint.z);
                double threshold = max((depth*sin(ang_res_y/2.0/180.0*M_PI)*2),eps_);

                search_method_->radiusSearch(input_cloud_->points[cloud_index],threshold,nn_indices,nn_distances);
                nn_size = nn_indices.size();
                // nn_size = radiusSearch(cloud_index, threshold, nn_indices, nn_distances);
                if (nn_size >= minPts_) {
                    for (int j = 0; j < nn_size; j++) {
                        if (types[nn_indices[j]] == UN_PROCESSED) {
                            
                            seed_queue.push_back(nn_indices[j]);
                            types[nn_indices[j]] = PROCESSING;
                        }
                    }
                }
                
                types[cloud_index] = PROCESSED;
                sq_idx++;
            }
            if (seed_queue.size() >= min_pts_per_cluster_ && seed_queue.size () <= max_pts_per_cluster_) {
                pcl::PointIndices r;
                r.indices.resize(seed_queue.size());
                for (int j = 0; j < seed_queue.size(); ++j) {
                    r.indices[j] = seed_queue[j];
                }
                // std::cout<<"r size "<<r.indices.size()<<std::endl;
                // These two lines should not be needed: (can anyone confirm?) -FF
                //Remove duplicate indexes
                std::sort (r.indices.begin (), r.indices.end ());
                r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());
                // std::cout<<"after erase r size "<<r.indices.size()<<std::endl;
                r.header = input_cloud_->header;
                cluster_indices.push_back (r);   // We could avoid a copy by working directly in the vector
            }
        } // for every point in input cloud
        std::sort (cluster_indices.rbegin (), cluster_indices.rend (), comparePointClusters);
    }

    void setClusterTolerance(double tolerance) {
        eps_ = tolerance; 
    }

    void setMinClusterSize (int min_cluster_size) { 
        min_pts_per_cluster_ = min_cluster_size; 
    }

    void setMaxClusterSize (int max_cluster_size) { 
        max_pts_per_cluster_ = max_cluster_size; 
    }
    
    void setCorePointMinPts(int core_point_min_pts) {
        minPts_ = core_point_min_pts;
    }

protected:
    PointCloudPtr input_cloud_;
    
    double eps_ {0.0};
    int minPts_ {1}; // not including the point itself.
    int min_pts_per_cluster_ {1};
    int max_pts_per_cluster_ {std::numeric_limits<int>::max()};

    KdFLANNPtr search_method_;

    virtual int radiusSearch(
        int index, double radius, std::vector<int> &k_indices,
        std::vector<float> &k_sqr_distances) const
    {
        k_indices.clear();
        k_sqr_distances.clear();
        k_indices.push_back(index);
        k_sqr_distances.push_back(0);
        int size = input_cloud_->points.size();
        double radius_square = radius * radius;
        for (int i = 0; i < size; i++) {
            if (i == index) {
                continue;
            }
            double distance_x = input_cloud_->points[i].x - input_cloud_->points[index].x;
            double distance_y = input_cloud_->points[i].y - input_cloud_->points[index].y;
            double distance_z = input_cloud_->points[i].z - input_cloud_->points[index].z;
            double distance_square = distance_x * distance_x + distance_y * distance_y + distance_z * distance_z;
            if (distance_square <= radius_square) {
                k_indices.push_back(i);
                k_sqr_distances.push_back(std::sqrt(distance_square));
            }
        }
        return k_indices.size();
    }
}; // class DBSCANCluster

#endif // DBSCAN_H
