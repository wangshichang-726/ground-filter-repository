#ifndef PMF_GROUND_FILTER_CORE_H_
#define PMF_GROUND_FILTER_CORE_H_

#include "../../common/lidar/point_cloud.h"
#include <string>
#include <utility>
#include <memory>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using tclPointXYZI = data_types::driver::lidar::PointCloud<pcl::PointXYZI>;

class PmfGroundFilter {
 public:
  PmfGroundFilter(ros::NodeHandle &nh);
  ~PmfGroundFilter();
  void Spin();
  void vProcess(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

 private:
  ros::Subscriber sub_point_cloud_;
  ros::Publisher pub_ground_, pub_no_ground_, pub_all_points_;
  std::string point_topic_;
  float f_max_window_size_;
  float f_max_distance_;
  float f_init_distance_;
  float f_cell_size_;
  float f_slope_;
  float f_base_;
  bool  bo_exponential_;
  bool  bo_negative_;
  bool  bo_extract_ground_;

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>
  progressiveMorphologicalFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud,
                                 float max_window_size, float max_distance,
                                 float init_distance, float cell_size,
                                 float slope, float base, bool exponential,
                                 bool negative, bool bo_extract_ground);
};

#endif // PMF_GROUND_FILTER_CORE_H_
