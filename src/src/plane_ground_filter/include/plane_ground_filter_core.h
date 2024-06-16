#pragma once
#include <string>
#include <ros/ros.h>

// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>

// using eigen lib
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>

namespace plane_ground_filter {

struct stPointcloudXYZI {
  PCL_ADD_POINT4D;                // quad-word XYZ
  float intensity;                // laser intensity reading
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

};  // namespace plane_ground_filter


POINT_CLOUD_REGISTER_POINT_STRUCT(plane_ground_filter::stPointcloudXYZI,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity))

// Customed Point Struct for holding clustered points
namespace ground_filter {
/** Euclidean Velodyne coordinate, including intensity and ring number, and label. */
struct stPointCloudXYZIRL {
  PCL_ADD_POINT4D;                 // quad-word XYZ
  float intensity;                 ///< laser intensity reading
  uint16_t ring;                   ///< laser ring number
  uint16_t label;                  ///< point label
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

};  // namespace ground_filter

#define PointCloudXYZIRL ground_filter::stPointCloudXYZIRL
#define PointcloudXYZI plane_ground_filter::stPointcloudXYZI
#define RUN pcl::PointCloud<PointCloudXYZIRL>

// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(ground_filter::stPointCloudXYZIRL,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(uint16_t, label, label))

using Eigen::JacobiSVD;
using Eigen::MatrixXf;
using Eigen::VectorXf;

class PlaneGroundFilter {
 public:
  PlaneGroundFilter(ros::NodeHandle &nh);
  ~PlaneGroundFilter();
  void Spin();

 private:
  ros::Subscriber sub_point_cloud_;
  ros::Publisher pub_ground_, pub_no_ground_, pub_all_points_;
  std::string point_topic_;

  int i_sensor_model_;
  double d_sensor_height_, d_clip_height_, d_min_distance_, d_max_distance_;
  int i_num_seg_ = 1;
  int i_num_iter_, i_num_lpr_;
  double d_th_seeds_, d_th_dist_;
  // Model parameter for ground plane fitting
  // The ground plane model is: ax+by+cz+d=0
  // Here normal:=[a,b,c], d=d
  // th_dist_d_ = threshold_dist - d
  float d_, th_dist_d_;
  MatrixXf normal_;

  // pcl::PointCloud<PointcloudXYZI>::Ptr g_seeds_pc(new pcl::PointCloud<PointcloudXYZI>());
  // pcl::PointCloud<PointcloudXYZI>::Ptr g_ground_pc(new pcl::PointCloud<PointcloudXYZI>());
  // pcl::PointCloud<PointcloudXYZI>::Ptr g_not_ground_pc(new pcl::PointCloud<PointcloudXYZI>());
  // pcl::PointCloud<PointCloudXYZIRL>::Ptr g_all_pc(new pcl::PointCloud<PointCloudXYZIRL>());

  pcl::PointCloud<PointcloudXYZI>::Ptr g_seeds_pc;
  pcl::PointCloud<PointcloudXYZI>::Ptr g_ground_pc;
  pcl::PointCloud<PointcloudXYZI>::Ptr g_not_ground_pc;
  pcl::PointCloud<PointCloudXYZIRL>::Ptr g_all_pc;

  void estimate_plane_(void);
  void extract_initial_seeds_(const pcl::PointCloud<PointcloudXYZI> &p_sorted);
  void post_process(const pcl::PointCloud<PointcloudXYZI>::Ptr in, const pcl::PointCloud<PointcloudXYZI>::Ptr out);
  void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud);
  void clip_above(const pcl::PointCloud<PointcloudXYZI>::Ptr in,
                  const pcl::PointCloud<PointcloudXYZI>::Ptr out);
  void remove_close_far_pt(const pcl::PointCloud<PointcloudXYZI>::Ptr in,
                          const pcl::PointCloud<PointcloudXYZI>::Ptr out);
};