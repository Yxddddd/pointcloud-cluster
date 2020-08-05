#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_


#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include "lidar_gnss_topological/cloud_msgs.h"

#include <opencv/cv.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#define PI 3.14159265

using namespace std;

typedef pcl::PointXYZI  PointType;

// // VLP-16
// extern const string lidarTopic = "/velodyne_points";
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 0.2;
// extern const float ang_res_y = 2.0;
// extern const float ang_bottom = 15.0+0.1;
// extern const int groundScanInd = 7;
 //HDL-64
extern const string lidarTopic = "/kitti/velo/pointcloud";
extern const int N_SCAN = 64;
extern const int Horizon_SCAN = 2250;
extern const float ang_res_x = 0.16;
extern const float ang_res_y = 0.427;
extern const float ang_bottom = 24.9;
extern const int groundScanInd = 50;

//lslidar
// extern const string lidarTopic = "/lslidar_point_cloud";
// extern const int N_SCAN = 32;
// extern const int Horizon_SCAN = 2000;
// extern const float ang_res_x = 0.18;
// extern const float ang_res_y = 1.0;
// extern const float ang_bottom = 16.1;
// extern const int groundScanInd = 15;

// HDL-32E
// extern const int N_SCAN = 32;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 41.33/float(N_SCAN-1);
// extern const float ang_bottom = 30.67;
// extern const int groundScanInd = 20;

extern const bool loopClosureEnableFlag = true;	//闭环检测标志位
extern const double mappingProcessInterval = 0.15;   //4hz??  0.2

extern const float scanPeriod = 0.1;
extern const int systemDelay = 0;
extern const int imuQueLength = 100;
extern const string imuTopic = "/imu/data";


extern const float sensorMountAngle = 0.0;
//extern const float segmentTheta = 1.0472;
// extern const float segmentTheta = 15.0/180.0*M_PI;
extern const int segmentValidPointNum = 5;
extern const int segmentValidLineNum = 3;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


extern const int edgeFeatureNum = 2;
extern const int surfFeatureNum = 4;
extern const int sectionsTotal = 6;
extern const float edgeThreshold = 0.1;
extern const float surfThreshold = 0.1;
extern const float nearestFeatureSearchSqDist = 25;

extern const float surroundingKeyframeSearchRadius = 50.0;
extern const int   surroundingKeyframeSearchNum = 50;

extern const float historyKeyframeSearchRadius = 5.0;
extern const int   historyKeyframeSearchNum = 25;
extern const float historyKeyframeFitnessScore = 0.3;

extern const float globalMapVisualizationSearchRadius = 500.0;

struct PointToLineDist
{
  PointToLineDist( pcl::PointXYZI& pointi, 
           pcl::PointXYZI& tripod1, 
           pcl::PointXYZI& tripod2): pointi(pointi),tripod1(tripod1),tripod2(tripod2) {}
           
  template <typename T>
  bool operator()(const T* const transform, T* residual) const 
  {
    T x = T(pointi.x);
    T y = T(pointi.y);
    T z = T(pointi.z);
    T x1 = T(tripod1.x);
    T y1 = T(tripod1.y);
    T z1 = T(tripod1.z);
    T x2 = T(tripod2.x);
    T y2 = T(tripod2.y);
    T z2 = T(tripod2.z);
    
    T s = T(10 * (pointi.intensity - int(pointi.intensity)));

    T rx = s * transform[0];  
    T ry = s * transform[1];
    T rz = s * transform[2];
    T tx = s * transform[3];
    T ty = s * transform[4];
    T tz = s * transform[5];
    
    T x01 = cos(rz) * (x - tx) + sin(rz) * (y - ty);
    T y01 = -sin(rz) * (x - tx) + cos(rz) * (y - ty);
    T z01 = (z - tz);

    T x02 = x01;
    T y02 = cos(rx) * y01 + sin(rx) * z01;
    T z02 = -sin(rx) * y01 + cos(rx) * z01;

    T x0 = cos(ry) * x02 - sin(ry) * z02;//投影到k开始时刻的点
    T y0 = y02;
    T z0 = sin(ry) * x02 + cos(ry) * z02;
    
    T a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
        * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
        + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
        * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
        + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
        * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));
        
    T l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));  
    T ld2 = a012 / l12;
    residual[0] = T(1.0) * ld2;

    return true;
    }
  
  static ceres::CostFunction* Create( pcl::PointXYZI& pointi, 
                    pcl::PointXYZI& tripod1, 
                    pcl::PointXYZI& tripod2) 
  {
    return (new ceres::AutoDiffCostFunction<PointToLineDist, 1, 6>(new PointToLineDist(pointi, tripod1, tripod2)));
    }

  const pcl::PointXYZI pointi;//原始点i
  const pcl::PointXYZI tripod1;//对应的边缘线上的点j
  const pcl::PointXYZI tripod2;//对应的边缘线上的点l
}; 
//角点优化时，使用xy和偏航角
class CornerCostFunction
  : public ceres::SizedCostFunction<1,6>{  //1,1声明残差和parameter的维度
public:
  CornerCostFunction(Eigen::Vector3d cp, Eigen::Vector3d lpj, Eigen::Vector3d lpl, double s) : cp_(cp), lpj_(lpj), lpl_(lpl), s_(s) {}
  virtual ~CornerCostFunction(){}
  //对于parameters 0-2 为位移量，3-5为角度值
  virtual bool Evaluate(double const* const* parameters,double *residuals, double **jacobians)const{
    Eigen::Vector3d tmp_cp;
    tmp_cp.x() = cp_.x() - s_*parameters[0][3];
    tmp_cp.y() = cp_.y() - s_*parameters[0][4];
    tmp_cp.z() = cp_.z() - s_*parameters[0][5];
    // Eigen::Vector3d lp = (Eigen::AngleAxisd(s_ * parameters[0][1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(s_ * parameters[0][0],
    //                      Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(s_*  parameters[0][2], Eigen::Vector3d::UnitZ())) * tmp_cp ; //转换后的点

    double rx = s_ * parameters[0][0];  
    double ry = s_ * parameters[0][1];
    double rz = s_ * parameters[0][2];
    double ttx = s_ * parameters[0][3];
    double tty = s_ * parameters[0][4];
    double ttz = s_ * parameters[0][5];

    double x01 = cos(rz) * (cp_.x() - ttx) + sin(rz) * (cp_.y() - tty);
    double y01 = -sin(rz) * (cp_.x() - ttx) + cos(rz) * (cp_.y() - tty);
    double z01 = (cp_.z() - ttz);

    double x02 = x01;
    double y02 = cos(rx) * y01 + sin(rx) * z01;
    double z02 = -sin(rx) * y01 + cos(rx) * z01;
    Eigen::Vector3d lp;
    lp.x() = cos(ry) * x02 - sin(ry) * z02;//投影到k开始时刻的点
    lp.y() = y02;
    lp.z() = sin(ry) * x02 + cos(ry) * z02;
    
    double k = std::sqrt(std::pow(lpj_.x() - lpl_.x(), 2) + std::pow(lpj_.y() - lpl_.y(), 2) + std::pow(lpj_.z() - lpl_.z(), 2)); //odom中角点距离残差的分母
    double a = (lp.y() - lpj_.y()) * (lp.z() - lpl_.z()) - (lp.z() - lpj_.z()) * (lp.y() - lpl_.y()); //m33
    double b = (lp.z() - lpj_.z()) * (lp.x() - lpl_.x()) - (lp.x() - lpj_.x()) * (lp.z() - lpl_.z()); //m22
    double c = (lp.x() - lpj_.x()) * (lp.y() - lpl_.y()) - (lp.y() - lpj_.y()) * (lp.x() - lpl_.x()); //m11
    double m = std::sqrt(a * a + b * b + c * c);

    residuals[0] = 1*m / k; //残差
    //对残差求雅克比
    double dm_dx = (b * (lpl_.z() - lpj_.z()) + c * (lpj_.y() - lpl_.y())) / m;   //误差对x求导
    double dm_dy = (a * (lpj_.z() - lpl_.z()) - c * (lpj_.x() - lpl_.x())) / m;
    double dm_dz = (-a * (lpj_.y() - lpl_.y()) - b * (lpj_.x() - lpl_.x())) / m;

    double srx = std::sin(parameters[0][0]);
    double crx = std::cos(parameters[0][0]);
    double sry = std::sin(parameters[0][1]);
    double cry = std::cos(parameters[0][1]);
    double srz = std::sin(parameters[0][2]);
    double crz = std::cos(parameters[0][2]);
    double tx = parameters[0][0];
    double ty = parameters[0][1];
    double tz = parameters[0][2];

    float dx_dy_yaw_0_0 = -crz*sry - cry*srx*srz; float dx_dy_yaw_0_1 = cry*crz*srx - sry*srz; float dx_dy_yaw_0_2 = crx*cry; 
    float dx_dy_yaw_x = tx*-dx_dy_yaw_0_0 + ty*-dx_dy_yaw_0_1 + tz*dx_dy_yaw_0_2;     //dui tx
    float dx_dy_yaw_2_0 = cry*crz - srx*sry*srz; float dx_dy_yaw_2_1 = cry*srz + crz*srx*sry; float dx_dy_yaw_2_2 = crx*sry; 
    float dx_dy_yaw_z = tz*dx_dy_yaw_2_2 - ty*dx_dy_yaw_2_1 - tx*dx_dy_yaw_2_0;
    float dx_dy = crx*srz;

    float dx_dx_pitch_0_0 = crx*sry*srz; float dx_dx_pitch_0_1 = crx*crz*sry; float dx_dx_pitch_0_2 = srx*sry; 
    float dx_dx_dx0 = tx*dx_dx_pitch_0_0 - ty*dx_dx_pitch_0_1 - tz*dx_dx_pitch_0_2;
    float dx_dx_pitch_1_0 = srx*srz; float dx_dx_pitch_1_1 = crz*srx; 
    float dx_dy_dx0 = ty*dx_dx_pitch_1_0 - tz*crx - tx*dx_dx_pitch_1_1;
    float dx_dx_pitch_2_0 = crx*cry*srz; float dx_dx_pitch_2_1 = crx*cry*crz; float dx_dx_pitch_2_2 = cry*srx; 
    float dx_dz_dx0 = tz*dx_dx_pitch_2_2 + ty*dx_dx_pitch_2_1 - tx*dx_dx_pitch_2_0;

    float dx_dz_roll_0_0 = -cry*srz - crz*srx*sry; float dx_dz_roll_0_1 = cry*crz - srx*sry*srz;
    float dx_dx_dz0 = tx*(-dx_dz_roll_0_0) - ty*dx_dz_roll_0_1;
    float dx_dz_roll_1_0 = -crx*crz; float dx_dz_roll_1_1 = crx*srz;
    float dx_dy_dz0 = ty*dx_dz_roll_1_1 + tx*(-dx_dz_roll_1_0);
    float dx_dz_roll_2_0 = cry*crz*srx - sry*srz;  float dx_dz_roll_2_1 = crz*sry + cry*srx*srz;
    float dx_dz_dz0 = tx * (-dx_dz_roll_2_0) - ty*(dx_dz_roll_2_1);
    if (jacobians && jacobians[0])
    {
      jacobians[0][0] = (-dx_dx_pitch_0_0*cp_.x()+dx_dx_pitch_0_1*cp_.y()+dx_dx_pitch_0_2*cp_.z() + dx_dx_dx0)*dm_dx
                        +(dx_dx_pitch_1_0*cp_.x()-dx_dx_pitch_1_1*cp_.y()+crx*cp_.z()+dx_dy_dx0)*dm_dy
                        +(dx_dx_pitch_2_0*cp_.x()-dx_dx_pitch_2_1*cp_.y()-dx_dx_pitch_2_2*cp_.z() + dx_dz_dx0)*dm_dz;

      jacobians[0][1] = (dx_dy_yaw_0_0*cp_.x()+dx_dy_yaw_0_1*cp_.y()-dx_dy_yaw_0_2*cp_.z() + dx_dy_yaw_x)*dm_dx
                        +(dx_dy_yaw_2_0*cp_.x() + dx_dy_yaw_2_1*cp_.y() - dx_dy_yaw_2_2*cp_.z() + dx_dy_yaw_z)*dm_dz;

      jacobians[0][2] = (dx_dz_roll_0_0*cp_.x() + dx_dz_roll_0_1*cp_.y() + dx_dx_dz0)*dm_dx
                        +(dx_dz_roll_1_0*cp_.x() - dx_dz_roll_1_1*cp_.y() + dx_dy_dz0)*dm_dy
                        +(dx_dz_roll_2_0*cp_.x() + dx_dz_roll_2_1*cp_.y() + dx_dz_dz0)*dm_dz;

      jacobians[0][3] = -dx_dy_yaw_2_0 * dm_dx + dx_dy *dm_dy + dx_dy_yaw_0_0*dm_dz;
      jacobians[0][4] = dx_dz_roll_0_0*dm_dx + dx_dz_roll_1_0*dm_dy + dx_dz_roll_2_0*dm_dz;
      jacobians[0][5] = dx_dy_yaw_2_2*dm_dx - srx*dm_dy -dx_dy_yaw_0_2*dm_dz;
    }

    return true;
  }
private:
  Eigen::Vector3d cp_;        // under t frame
  Eigen::Vector3d lpj_, lpl_; // under t-1 frame
  double s_;
};
//laserodometry中的角点
class SurfCostFunction
  : public ceres::SizedCostFunction<1,6>{  //1,1声明残差和parameter的维度
public:
  SurfCostFunction(Eigen::Vector3d cp, Eigen::Vector3d lpj, Eigen::Vector3d lpl, 
          Eigen::Vector3d lpm, double s) : cp_(cp), lpj_(lpj), lpl_(lpl), lpm_(lpm), s_(s) {}
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians)const{
    Eigen::Vector3d tmp_cp;
    tmp_cp.x() = cp_.x() - s_*parameters[0][3];
    tmp_cp.y() = cp_.y() - s_*parameters[0][4];
    tmp_cp.z() = cp_.z() - s_*parameters[0][5];

    // Eigen::Vector3d lp = (Eigen::AngleAxisd(s_ * parameters[0][1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(s_ * parameters[0][0],
    //                      Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(s_*  parameters[0][2], Eigen::Vector3d::UnitZ())) * tmp_cp ; //转换后的点
    double rx = s_ * parameters[0][0];  
    double ry = s_ * parameters[0][1];
    double rz = s_ * parameters[0][2];
    double ttx = s_ * parameters[0][3];
    double tty = s_ * parameters[0][4];
    double ttz = s_ * parameters[0][5];

    double x01 = cos(rz) * (cp_.x() - ttx) + sin(rz) * (cp_.y() - tty);
    double y01 = -sin(rz) * (cp_.x() - ttx) + cos(rz) * (cp_.y() - tty);
    double z01 = (cp_.z() - ttz);

    double x02 = x01;
    double y02 = cos(rx) * y01 + sin(rx) * z01;
    double z02 = -sin(rx) * y01 + cos(rx) * z01;
    Eigen::Vector3d lp;
    lp.x() = cos(ry) * x02 - sin(ry) * z02;//投影到k开始时刻的点
    lp.y() = y02;
    lp.z() = sin(ry) * x02 + cos(ry) * z02;
    
    // Eigen::Vector3d lp = (Eigen::AngleAxisd(parameters[0][1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(parameters[0][0],
    //                      Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(parameters[0][2], Eigen::Vector3d::UnitZ())) * cp_ 
    //                      + Eigen::Vector3d(parameters[0][3], parameters[0][4], parameters[0][5]); //转换后的点
    double a = (lpj_.y() - lpl_.y()) * (lpj_.z() - lpm_.z()) - (lpj_.z() - lpl_.z()) * (lpj_.y() - lpm_.y());
    double b = (lpj_.z() - lpl_.z()) * (lpj_.x() - lpm_.x()) - (lpj_.x() - lpl_.x()) * (lpj_.z() - lpm_.z());
    double c = (lpj_.x() - lpl_.x()) * (lpj_.y() - lpm_.y()) - (lpj_.y() - lpl_.y()) * (lpj_.x() - lpm_.x());
    a *= a;
    b *= b;
    c *= c;
    double m = std::sqrt(std::pow((lp.x() - lpj_.x()), 2) * a + std::pow((lp.y() - lpj_.y()), 2) * b + std::pow((lp.z() - lpj_.z()), 2) * c);
    double k = std::sqrt(a + b + c);

    residuals[0] = m / k;
    //std::cout<<residuals[0]<<std::endl;

    double tmp = m * k;

    double dm_dx = ((lp.x() - lpj_.x()) * a) / tmp;
    double dm_dy = ((lp.y() - lpj_.y()) * b) / tmp;
    double dm_dz = ((lp.z() - lpj_.z()) * c) / tmp;

    double srx = std::sin(parameters[0][0]);
    double crx = std::cos(parameters[0][0]);
    double sry = std::sin(parameters[0][1]);
    double cry = std::cos(parameters[0][1]);
    double srz = std::sin(parameters[0][2]);
    double crz = std::cos(parameters[0][2]);
    double tx = parameters[0][0];
    double ty = parameters[0][1];
    double tz = parameters[0][2];

    float dx_dy_yaw_0_0 = -crz*sry - cry*srx*srz; float dx_dy_yaw_0_1 = cry*crz*srx - sry*srz; float dx_dy_yaw_0_2 = crx*cry; 
    float dx_dy_yaw_x = tx*-dx_dy_yaw_0_0 + ty*-dx_dy_yaw_0_1 + tz*dx_dy_yaw_0_2;     //dui tx
    float dx_dy_yaw_2_0 = cry*crz - srx*sry*srz; float dx_dy_yaw_2_1 = cry*srz + crz*srx*sry; float dx_dy_yaw_2_2 = crx*sry; 
    float dx_dy_yaw_z = tz*dx_dy_yaw_2_2 - ty*dx_dy_yaw_2_1 - tx*dx_dy_yaw_2_0;
    float dx_dy = crx*srz;

    float dx_dx_pitch_0_0 = crx*sry*srz; float dx_dx_pitch_0_1 = crx*crz*sry; float dx_dx_pitch_0_2 = srx*sry; 
    float dx_dx_dx0 = tx*dx_dx_pitch_0_0 - ty*dx_dx_pitch_0_1 - tz*dx_dx_pitch_0_2;
    float dx_dx_pitch_1_0 = srx*srz; float dx_dx_pitch_1_1 = crz*srx; 
    float dx_dy_dx0 = ty*dx_dx_pitch_1_0 - tz*crx - tx*dx_dx_pitch_1_1;
    float dx_dx_pitch_2_0 = crx*cry*srz; float dx_dx_pitch_2_1 = crx*cry*crz; float dx_dx_pitch_2_2 = cry*srx; 
    float dx_dz_dx0 = tz*dx_dx_pitch_2_2 + ty*dx_dx_pitch_2_1 - tx*dx_dx_pitch_2_0;

    float dx_dz_roll_0_0 = -cry*srz - crz*srx*sry; float dx_dz_roll_0_1 = cry*crz - srx*sry*srz;
    float dx_dx_dz0 = tx*(-dx_dz_roll_0_0) - ty*dx_dz_roll_0_1;
    float dx_dz_roll_1_0 = -crx*crz; float dx_dz_roll_1_1 = crx*srz;
    float dx_dy_dz0 = ty*dx_dz_roll_1_1 + tx*(-dx_dz_roll_1_0);
    float dx_dz_roll_2_0 = cry*crz*srx - sry*srz;  float dx_dz_roll_2_1 = crz*sry + cry*srx*srz;
    float dx_dz_dz0 = tx * (-dx_dz_roll_2_0) - ty*(dx_dz_roll_2_1);
    if (jacobians && jacobians[0])
    {
      jacobians[0][0] = (-dx_dx_pitch_0_0*cp_.x()+dx_dx_pitch_0_1*cp_.y()+dx_dx_pitch_0_2*cp_.z() + dx_dx_dx0)*dm_dx
                        +(dx_dx_pitch_1_0*cp_.x()-dx_dx_pitch_1_1*cp_.y()+crx*cp_.z()+dx_dy_dx0)*dm_dy
                        +(dx_dx_pitch_2_0*cp_.x()-dx_dx_pitch_2_1*cp_.y()-dx_dx_pitch_2_2*cp_.z() + dx_dz_dx0)*dm_dz;

      jacobians[0][1] = (dx_dy_yaw_0_0*cp_.x()+dx_dy_yaw_0_1*cp_.y()-dx_dy_yaw_0_2*cp_.z() + dx_dy_yaw_x)*dm_dx
                        +(dx_dy_yaw_2_0*cp_.x() + dx_dy_yaw_2_1*cp_.y() - dx_dy_yaw_2_2*cp_.z() + dx_dy_yaw_z)*dm_dz;

      jacobians[0][2] = (dx_dz_roll_0_0*cp_.x() + dx_dz_roll_0_1*cp_.y() + dx_dx_dz0)*dm_dx
                        +(dx_dz_roll_1_0*cp_.x() - dx_dz_roll_1_1*cp_.y() + dx_dy_dz0)*dm_dy
                        +(dx_dz_roll_2_0*cp_.x() + dx_dz_roll_2_1*cp_.y() + dx_dz_dz0)*dm_dz;

      jacobians[0][3] = -dx_dy_yaw_2_0 * dm_dx + dx_dy *dm_dy + dx_dy_yaw_0_0*dm_dz;
      jacobians[0][4] = dx_dz_roll_0_0*dm_dx + dx_dz_roll_1_0*dm_dy + dx_dz_roll_2_0*dm_dz;
      jacobians[0][5] = dx_dy_yaw_2_2*dm_dx - srx*dm_dy -dx_dy_yaw_0_2*dm_dz;
    }
    return true;
  }

private:
  Eigen::Vector3d cp_;
  Eigen::Vector3d lpj_, lpl_, lpm_;
  double s_;
};

//laserodometry中的面点
struct LidarPlaneFactor
{
  LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
           Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
    : curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
      last_point_m(last_point_m_), s(s_)
  {
    ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);//（xk-1,j - xk-1l)X(xk-1,j - xk-1,m)
    ljm_norm.normalize();
  }

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const
  {

    Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
    Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
    //Eigen::Matrix<T, 3, 1> lpl{T(last_point_l.x()), T(last_point_l.y()), T(last_point_l.z())};
    //Eigen::Matrix<T, 3, 1> lpm{T(last_point_m.x()), T(last_point_m.y()), T(last_point_m.z())};
    Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

    //Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
    Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
    Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
    q_last_curr = q_identity.slerp(T(s), q_last_curr);      //slerp插值
    Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

    Eigen::Matrix<T, 3, 1> lp;
    lp = q_last_curr * cp + t_last_curr;  //通过当前时刻的旋转平移矩阵得到转换后的坐标
   // lp = q_last_curr*(cp+t_last_curr);

    //std::cout<<"hpp file"<<std::endl;
    //std::cout<<lp<<std::endl;

    residual[0] = (lp - lpj).dot(ljm);    //dot为点乘 由于分子中叉乘部分使用了归一化，所以相对于论文中的公式，不用再除以分母，获得的就是点到平面的距离

    return true;
  }
  //构建面点时需要上一帧三个点和当前帧一个点
  static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
                     const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_,
                     const double s_)
  {
    return (new ceres::AutoDiffCostFunction<
        LidarPlaneFactor, 1, 4, 3>(
      new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_, s_)));//residual是一维q为四维，t是三维
  }

  Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
  Eigen::Vector3d ljm_norm;
  double s;
};
//mapping中面点的误差函数构建
struct LidarPlaneNormFactor
{

  LidarPlaneNormFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_,
             double negative_OA_dot_norm_) : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_),
                             negative_OA_dot_norm(negative_OA_dot_norm_) {}

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const
  {
    Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
    Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
    Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
    Eigen::Matrix<T, 3, 1> point_w;
    point_w = q_w_curr * cp + t_w_curr;     //当前点的坐标

    Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));
    residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);    //点到平面的距离即为残差向量
    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_,
                     const double negative_OA_dot_norm_)
  {
    return (new ceres::AutoDiffCostFunction<
        LidarPlaneNormFactor, 1, 4, 3>(
      new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)));
  }

  Eigen::Vector3d curr_point;
  Eigen::Vector3d plane_unit_norm;
  double negative_OA_dot_norm;
};


struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

#endif
