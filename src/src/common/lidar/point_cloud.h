// point cloud library
#include <pcl/point_cloud.h>
// common data
#include <string>

namespace data_types {
namespace driver {
namespace lidar {


struct Time {
  unsigned int sec;
  unsigned int nsec;
};

struct Header {
  unsigned int seq;
  Time time_stamp;
  std::string frame_id;
};

template<typename PointT>
struct PointCloud {
  // constructor
  PointCloud()
  :point_cloud_ptr(new pcl::PointCloud<PointT>()) {}
  ~PointCloud() {}
  // variable
  Header header;
  boost::shared_ptr<pcl::PointCloud<PointT>> point_cloud_ptr;
};

}  // namespace lidar
}  // namespace driver
}  // namespace data_types