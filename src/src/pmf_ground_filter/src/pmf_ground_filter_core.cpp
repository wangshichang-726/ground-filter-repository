#include "pmf_ground_filter_core.h"

PmfGroundFilter::PmfGroundFilter(ros::NodeHandle &nh) {
    std::string input_topic;
    nh.getParam("input_topic", input_topic);
    sub_point_cloud_ = nh.subscribe(input_topic, 10, &PmfGroundFilter::vProcess, this);

    std::string no_ground_topic, ground_topic, all_points_topic;
    nh.getParam("no_ground_point_topic", no_ground_topic);
    nh.getParam("ground_point_topic", ground_topic);
    nh.getParam("all_points_topic", all_points_topic);

    nh.getParam("max_window_size", f_max_window_size_);
    nh.getParam("max_distance", f_max_distance_);
    nh.getParam("init_distance", f_init_distance_);
    nh.getParam("cell_size", f_cell_size_);
    nh.getParam("slope", f_slope_);
    nh.getParam("base", f_base_);
    nh.getParam("negative", bo_negative_);
    nh.getParam("extract_ground", bo_extract_ground_);

    pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>(ground_topic, 10);
    pub_no_ground_ = nh.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 10);
    pub_all_points_ = nh.advertise<sensor_msgs::PointCloud2>(all_points_topic, 10);

    ros::spin();
}

PmfGroundFilter::~PmfGroundFilter() {}

void PmfGroundFilter::Spin() {
    ros::spin();
}

void PmfGroundFilter::vProcess(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_msg(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*in_cloud_ptr, *cloud_msg);

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> lidar_msg;
    if (!cloud_msg->empty()) {
        lidar_msg = progressiveMorphologicalFilter(cloud_msg, f_max_window_size_, f_max_distance_,
                                                   f_init_distance_, f_cell_size_, f_slope_, f_base_,
                                                   bo_exponential_, bo_negative_, bo_extract_ground_);
    } else {
        lidar_msg.first = cloud_msg;
        lidar_msg.second = cloud_msg;
    }

    sensor_msgs::PointCloud2 ground_msg, no_ground_msg, all_points_msg;
    pcl::toROSMsg(*lidar_msg.second, ground_msg);
    pcl::toROSMsg(*lidar_msg.first, no_ground_msg);
    pcl::toROSMsg(*cloud_msg, all_points_msg);

    ground_msg.header.stamp = in_cloud_ptr->header.stamp;
    ground_msg.header.frame_id = in_cloud_ptr->header.frame_id;
    no_ground_msg.header.stamp = in_cloud_ptr->header.stamp;
    no_ground_msg.header.frame_id = in_cloud_ptr->header.frame_id;
    all_points_msg.header.stamp = in_cloud_ptr->header.stamp;
    all_points_msg.header.frame_id = in_cloud_ptr->header.frame_id;

    pub_ground_.publish(ground_msg);
    pub_no_ground_.publish(no_ground_msg);
    pub_all_points_.publish(all_points_msg);
}

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>
PmfGroundFilter::progressiveMorphologicalFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud,
                                                float max_window_size, float max_distance,
                                                float init_distance, float cell_size,
                                                float slope, float base, bool exponential,
                                                bool negative, bool bo_extract_ground) {
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> result;

    pcl::PointIndices::Ptr groundIndices(new pcl::PointIndices);
    pcl::ApproximateProgressiveMorphologicalFilter<pcl::PointXYZI> pmf;
    pmf.setInputCloud(original_cloud);
    pmf.setMaxWindowSize(max_window_size);
    pmf.setMaxDistance(max_distance);
    pmf.setInitialDistance(init_distance);
    pmf.setCellSize(cell_size);
    pmf.setSlope(slope);
    pmf.setBase(base);
    pmf.setExponential(exponential);
    pmf.extract(groundIndices->indices);

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(original_cloud);
    extract.setIndices(groundIndices);
    extract.setNegative(negative);

    result.first.reset(new pcl::PointCloud<pcl::PointXYZI>);
    result.second.reset(new pcl::PointCloud<pcl::PointXYZI>);

    extract.filter(*result.first);
    if (bo_extract_ground) {
        extract.setNegative(!negative);
        extract.filter(*result.second);
    } else {
        *result.second = *result.first;
    }

    return result;
}
