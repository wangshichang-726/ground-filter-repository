#include "pmf_ground_filter_core.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "pmf_ground_filter");

    ros::NodeHandle nh("~");

    PmfGroundFilter pmf_filter(nh);

    // 调用 Spin 函数以保持节点运行
    pmf_filter.Spin();

    return 0;
}