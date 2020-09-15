#ifndef MAP_BUILDER_H
#define MAP_BUILDER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <chrono>
#include <fstream>
#include <mutex>
#include "math_func.h"
#include "key_frame.h"
#include "Transform.h"

namespace encode
{

class MapBuilder
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapBuilder();
    ~MapBuilder() {}

    void addPointCloud_odometry_only(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point_cloud, Transform& delta_pose);
    void saveOdometry();

    Eigen::Matrix4f getTransformation() { return odometry_pose_; }
    void getMap(sensor_msgs::PointCloud2& map_msg)
    {
        std::unique_lock<std::mutex> locker(map_mutex_);
        pcl::toROSMsg(map_, map_msg);
    }

private:
    KeyFrame::Ptr addVertex(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point_cloud, Eigen::Matrix4f& pose);
    void downSampleMap();

private:
    pcl::PointCloud<pcl::PointXYZI> map_;
    std::vector<KeyFrame::Ptr> key_frames_;

    int sequence_count_;

    Eigen::Matrix4f pose_;
    Eigen::Matrix4f odometry_pose_;
   
    int map_update_count_;

    std::mutex map_mutex_;
};

class CloudDownSample {
 public:
  struct pointAttr {
    double x;
    double y;
    double z;
    double intensity;
    uint32_t num;
  };
  CloudDownSample(double resolution);
  virtual ~CloudDownSample();
  void inputCloud(const pcl::PointCloud<pcl::PointXYZI> &cloud);
  void getDownSampledCloud(pcl::PointCloud<pcl::PointXYZI> &downCloud);
  void clear();

 protected:
  double mResolution;
  std::map<double, std::map<double, std::map<double, pointAttr>>> mMapGrid;
};

} // namespace encode

#endif // MAP_BUILDER_H
