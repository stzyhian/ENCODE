#ifndef ENCODE_ROS_H
#define ENCODE_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <thread>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include "map_builder.h"

namespace encode
{

typedef Eigen::Matrix<float, 6, 1> Vector6f;

class EncodeRos
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EncodeRos();
    ~EncodeRos() {}

private:
    Vector6f getPose(const Eigen::Matrix4f& T);
    void publishMap();
    void publishPose(const Vector6f& pose, const ros::Time& t);
    void publishPath(const Vector6f& pose, const ros::Time& t);
    void publishTf(const Vector6f& pose, const ros::Time& t);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);
    void dispatchPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, Transform& delta_pose);
    bool saveOdometryCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

private:
    ros::Publisher map_pub_;
    ros::Publisher path_pub_;
    ros::Publisher pose_pub_;
    ros::ServiceServer save_odometry_srv_;
    ros::Subscriber point_cloud_sub_;
    tf::TransformBroadcaster tf_broadcaster_;
    nav_msgs::Path path_msg_;

    std::string base_frame_;
    std::string map_frame_;
    double min_scan_distance_;
    double max_scan_distance_;

    encode::MapBuilder map_builder_;
};

} // namespace encode

#endif // ENCODE_ROS_H
