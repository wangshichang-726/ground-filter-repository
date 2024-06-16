#include "plane_ground_filter_core.h"

/*
    @brief 比较两个点的z坐标
    @return 根据z轴排序
*/
bool point_cmp(PointcloudXYZI a, PointcloudXYZI b) {
    return a.z < b.z;
}

/*
    @brief 初始化ROS节点
    @return 开始处理
*/
PlaneGroundFilter::PlaneGroundFilter(ros::NodeHandle &nh) {
    std::string input_topic;
    nh.getParam("input_topic", input_topic);
    sub_point_cloud_ = nh.subscribe("/lidar_aeb/raw_points", 10, &PlaneGroundFilter::point_cb, this);
    // sub_point_cloud_ = nh.subscribe("/rslidar_points", 10, &PlaneGroundFilter::point_cb, this);

    // init publisher
    std::string no_ground_topic, ground_topic, all_points_topic;

    nh.getParam("no_ground_point_topic", no_ground_topic);
    nh.getParam("ground_point_topic", ground_topic);
    nh.getParam("all_points_topic", all_points_topic);

    nh.getParam("clip_height", d_clip_height_);
    ROS_INFO("clip_height: %f", d_clip_height_);
    nh.getParam("sensor_height", d_sensor_height_);
    ROS_INFO("sensor_height: %f", d_sensor_height_);
    nh.getParam("min_distance", d_min_distance_);
    ROS_INFO("min_distance: %f", d_min_distance_);
    nh.getParam("max_distance", d_max_distance_);
    ROS_INFO("max_distance: %f", d_max_distance_);

    nh.getParam("sensor_model", i_sensor_model_);
    ROS_INFO("sensor_model: %d", i_sensor_model_);
    nh.getParam("num_iter", i_num_iter_);
    ROS_INFO("num_iter: %d", i_num_iter_);
    nh.getParam("num_lpr", i_num_lpr_);
    ROS_INFO("num_lpr: %d", i_num_lpr_);
    nh.getParam("th_seeds", d_th_seeds_);
    ROS_INFO("th_seeds: %f", d_th_seeds_);
    nh.getParam("th_dist", d_th_dist_);
    ROS_INFO("th_dist: %f", d_th_dist_);

    pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>(ground_topic, 10);
    pub_no_ground_ = nh.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 10);
    pub_all_points_ = nh.advertise<sensor_msgs::PointCloud2>(all_points_topic, 10);

    g_seeds_pc = pcl::PointCloud<PointcloudXYZI>::Ptr(new pcl::PointCloud<PointcloudXYZI>);
    g_ground_pc = pcl::PointCloud<PointcloudXYZI>::Ptr(new pcl::PointCloud<PointcloudXYZI>);
    g_not_ground_pc = pcl::PointCloud<PointcloudXYZI>::Ptr(new pcl::PointCloud<PointcloudXYZI>);
    g_all_pc = pcl::PointCloud<PointCloudXYZIRL>::Ptr(new pcl::PointCloud<PointCloudXYZIRL>);

    ros::spin();
}

PlaneGroundFilter::~PlaneGroundFilter() {}

void PlaneGroundFilter::Spin() { }

/*
    @brief 通过给特定点索引，过滤对应索引，过滤z轴大于d_clip_height_=3的点
    @return 高度过滤后的点云
*/
void PlaneGroundFilter::clip_above(const pcl::PointCloud<PointcloudXYZI>::Ptr in,
                                   const pcl::PointCloud<PointcloudXYZI>::Ptr out) {
    pcl::ExtractIndices<PointcloudXYZI> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
    #pragma omp for
    for (size_t i = 0; i < in->points.size(); i++) {
        if (in->points[i].z > d_clip_height_) {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true);  // ture to remove the indices
    cliper.filter(*out);
}

/*
    @brief 通过欧式距离范围内，过滤距离过远点，0.2～45
    @return 范围过滤后的点云
*/
void PlaneGroundFilter::remove_close_far_pt(const pcl::PointCloud<PointcloudXYZI>::Ptr in,
                                            const pcl::PointCloud<PointcloudXYZI>::Ptr out) {
    pcl::ExtractIndices<PointcloudXYZI> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
    #pragma omp for
    for (size_t i = 0; i < in->points.size(); i++) {
        double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);

        if ((distance < d_min_distance_) || (distance > d_max_distance_)) {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true);  // ture to remove the indices
    cliper.filter(*out);
}

/*
    @brief 平面估计算法，用于从给定地面点云中估计平面模型的参数
    具体来说，对协方差进行奇异值分解（SVD），从而得到平面的法向量和距离参数

    @param g_ground_pc:global ground pointcloud ptr.

    @return 返回平面方程参数

*/
void PlaneGroundFilter::estimate_plane_(void) {
    Eigen::Matrix3f cov;  // 3X3协方差矩阵
    Eigen::Vector4f pc_mean;  // 点云的均值向量，4维（包含奇次坐标
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);  // 计算点云的协方差矩阵和点云均值
    // 对协方差矩阵进行奇异值分解
    JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    // 使用最小的奇异值对应的奇异向量作为法向量
    normal_ = (svd.matrixU().col(2));
    // 提取地面种子点的均值（前三维）
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    // 根据公式 normal.T * [x, y, z] = -d 计算平面参数 d_
    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
    // 设置距离阈值
    th_dist_d_ = d_th_dist_ - d_;

    // 返回平面方程参数
}

/*
    @brief 从输入的点云数据中提取初始种子点，用于后续的地面分割算法.
    该函数根据高度信息筛选出地面种子点
    This function will set the `g_ground_pc` to `g_seed_pc`.
    @param p_sorted: sorted pointcloud

    @param ::i_num_lpr_: 最低点代表的点的数量=20
    @param ::d_th_seeds_: 种子点的高度阈值
    @param ::

    @return 返回种子点云g_seeds_pc
*/
void PlaneGroundFilter::extract_initial_seeds_(const pcl::PointCloud<PointcloudXYZI> &p_sorted) {
    double d_ground_height_sum_ = 0;
    int d_ground_points_cnt_ = 0;
    // Calculate the mean height value.
    for (int i = 0; i < p_sorted.points.size() && d_ground_points_cnt_ < i_num_lpr_; i++) {
        d_ground_height_sum_ += p_sorted.points[i].z;
        d_ground_points_cnt_++;
    }
    double lpr_height = d_ground_points_cnt_ != 0 ? d_ground_height_sum_ / d_ground_points_cnt_ : 0;
    g_seeds_pc->clear();
    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++) {
        if (p_sorted.points[i].z < lpr_height + d_th_seeds_) {
            g_seeds_pc->points.push_back(p_sorted.points[i]);
        }
    }
    // return seeds points
}

void PlaneGroundFilter::post_process(const pcl::PointCloud<PointcloudXYZI>::Ptr in, const pcl::PointCloud<PointcloudXYZI>::Ptr out) {
    pcl::PointCloud<PointcloudXYZI>::Ptr cliped_pc_ptr(new pcl::PointCloud<PointcloudXYZI>);
    clip_above(in, cliped_pc_ptr);
    pcl::PointCloud<PointcloudXYZI>::Ptr remove_close(new pcl::PointCloud<PointcloudXYZI>);
    remove_close_far_pt(cliped_pc_ptr, out);
}

void PlaneGroundFilter::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr) {
    // 1.Msg to pointcloud
    pcl::PointCloud<PointcloudXYZI> laserCloudIn;
    pcl::fromROSMsg(*in_cloud_ptr, laserCloudIn);

    pcl::PointCloud<PointcloudXYZI> laserCloudIn_org;
    pcl::fromROSMsg(*in_cloud_ptr, laserCloudIn_org);
    // For mark ground points and hold all points
    PointCloudXYZIRL point;

    for (size_t i = 0; i < laserCloudIn.points.size(); i++) {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        point.intensity = laserCloudIn.points[i].intensity;
        point.label = 0u;  // 0 means uncluster
        g_all_pc->points.push_back(point);
    }
    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn,indices);
    //  2.Sort on Z-axis value.
    sort(laserCloudIn.points.begin(), laserCloudIn.end(), point_cmp);
    // 3.Error point removal
    // As there are some error mirror reflection under the ground,
    // here regardless point under 2* sensor_height
    // Sort point according to height, here uses z-axis in default
    pcl::PointCloud<PointcloudXYZI>::iterator it = laserCloudIn.points.begin();
    for (int i = 0; i < laserCloudIn.points.size(); i++) {
        if (laserCloudIn.points[i].z < -1.5 * d_sensor_height_) {
            it++;
        } else {
            break;
        }
    }
    laserCloudIn.points.erase(laserCloudIn.points.begin(), it);
    // 4. Extract init ground seeds.
    extract_initial_seeds_(laserCloudIn);
    g_ground_pc = g_seeds_pc;
    // 5. Ground plane fitter mainloop
    for (int i = 0; i < i_num_iter_; i++) {
        estimate_plane_();
        g_ground_pc->clear();
        g_not_ground_pc->clear();

        // pointcloud to matrix
        MatrixXf points(laserCloudIn_org.points.size(), 3);
        int j = 0;
        for (auto p : laserCloudIn_org.points) {
            points.row(j++) << p.x, p.y, p.z;
        }
        // ground plane model
        VectorXf result = points * normal_;
        // threshold filter
        for (int r = 0; r < result.rows(); r++) {
            if (result[r] < th_dist_d_) {
                g_all_pc->points[r].label = 1u;  // means ground
                g_ground_pc->points.push_back(laserCloudIn_org[r]);
            } else {
                g_all_pc->points[r].label = 0u;  // means not ground and non clusterred
                g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
        }
    }

    pcl::PointCloud<PointcloudXYZI>::Ptr final_no_ground(new pcl::PointCloud<PointcloudXYZI>);
    post_process(g_not_ground_pc, final_no_ground);

    // ROS_INFO_STREAM("origin: "<<g_not_ground_pc->points.size()<<" post_process: "<<final_no_ground->points.size());

    // publish ground points
    sensor_msgs::PointCloud2 ground_msg;
    pcl::toROSMsg(*g_ground_pc, ground_msg);
    ground_msg.header.stamp = in_cloud_ptr->header.stamp;
    ground_msg.header.frame_id = in_cloud_ptr->header.frame_id;
    pub_ground_.publish(ground_msg);

    // publish not ground pointsA
    sensor_msgs::PointCloud2 groundless_msg;
    pcl::toROSMsg(*final_no_ground, groundless_msg);
    groundless_msg.header.stamp = in_cloud_ptr->header.stamp;
    groundless_msg.header.frame_id = in_cloud_ptr->header.frame_id;
    pub_no_ground_.publish(groundless_msg);

    // publish all points
    sensor_msgs::PointCloud2 all_points_msg;
    pcl::toROSMsg(*g_all_pc, all_points_msg);
    all_points_msg.header.stamp = in_cloud_ptr->header.stamp;
    all_points_msg.header.frame_id = in_cloud_ptr->header.frame_id;
    pub_all_points_.publish(all_points_msg);
    g_all_pc->clear();
}
