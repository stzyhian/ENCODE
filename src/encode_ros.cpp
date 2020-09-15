#include "encode_ros.h"
#include "Transform.h"
#include <pcl_conversions/pcl_conversions.h>

using namespace encode;

EncodeRos::EncodeRos()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string point_cloud_topic;

    private_nh.param("base_frame", base_frame_, std::string("base_link"));
    private_nh.param("map_frame", map_frame_, std::string("map"));
    private_nh.param("point_cloud_topic", point_cloud_topic, std::string("velodyne_points"));
    private_nh.param("min_scan_distance", min_scan_distance_, 0.0);
    private_nh.param("max_scan_distance", max_scan_distance_, 40.0);

    map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_map", 1, true);
    path_pub_ = nh.advertise<nav_msgs::Path>("path", 1, true);
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("pose", 1, true);

    save_odometry_srv_ = nh.advertiseService("save_odometry", &EncodeRos::saveOdometryCallback, this);
    point_cloud_sub_ = nh.subscribe(point_cloud_topic, 10000, &EncodeRos::pointCloudCallback, this);
}

Vector6f EncodeRos::getPose(const Eigen::Matrix4f& T)
{
    Vector6f pose;
    pose(0) = T(0, 3);
    pose(1) = T(1, 3);
    pose(2) = T(2, 3);

    tf::Matrix3x3 R;
    double roll, pitch, yaw;
    R.setValue(T(0, 0), T(0, 1), T(0, 2),
               T(1, 0), T(1, 1), T(1, 2),
               T(2, 0), T(2, 1), T(2, 2));
    R.getRPY(roll, pitch, yaw);
    pose(3) = roll;
    pose(4) = pitch;
    pose(5) = yaw;

    return pose;
}

bool EncodeRos::saveOdometryCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    map_builder_.saveOdometry();
    return true;
}

static const double Ang2Rad = 0.01745329251994;
Transform getTransformFromRPYT(double x, double y, double z, double yaw,
                               double pitch, double roll) {
  Translation t(x, y, z);
  Eigen::AngleAxisd rollAngle(roll * Ang2Rad, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitchAngle(pitch * Ang2Rad, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd yawAngle(yaw * Ang2Rad, Eigen::Vector3d::UnitZ());
  Rotation r = rollAngle * pitchAngle * yawAngle;
  return Transform(t, r);
}

void EncodeRos::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*point_cloud_msg, *point_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_point_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for(const pcl::PointXYZI& point: point_cloud->points) {
        double r = sqrt(square(point.x) + square(point.y));
        if (r > min_scan_distance_ && r < max_scan_distance_) {
            clipped_point_cloud->push_back(point);
        }
    }

    std::string pose_str = point_cloud_msg->header.frame_id;
    Transform delta_pose;
    double x, y, z, yaw, pitch, roll;

    sscanf(pose_str.c_str(), "[%lf %lf %lf %lf %lf %lf]", &x,&y,&z,&yaw,&pitch,&roll);
    delta_pose = getTransformFromRPYT(x, y, z, yaw, pitch, roll);

    dispatchPointCloud(clipped_point_cloud, delta_pose);
}

void EncodeRos::dispatchPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, Transform& delta_pose)
{
    map_builder_.addPointCloud_odometry_only(cloud, delta_pose);

    Vector6f pose = getPose(map_builder_.getTransformation());
    publishPose(pose, ros::Time::now());
    publishTf(pose, ros::Time::now());
    publishPath(pose, ros::Time::now());
    publishMap();
}

void EncodeRos::publishPose(const Vector6f& pose, const ros::Time& t)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = t;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.position.x = pose(0);
    pose_msg.pose.position.y = pose(1);
    pose_msg.pose.position.z = pose(2);
    tf::Quaternion q;
    q.setRPY(pose(3), pose(4), pose(5));
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    pose_pub_.publish(pose_msg);
}

void EncodeRos::publishPath(const Vector6f& pose, const ros::Time& t)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = t;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.position.x = pose(0);
    pose_msg.pose.position.y = pose(1);
    pose_msg.pose.position.z = pose(2);

    tf::Quaternion q;
    q.setRPY(pose(3), pose(4), pose(5));
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    path_msg_.poses.push_back(pose_msg);

    path_msg_.header.stamp = t;
    path_msg_.header.frame_id = map_frame_;
    path_pub_.publish(path_msg_);
}

void EncodeRos::publishTf(const Vector6f& pose, const ros::Time& t)
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose(0), pose(1), pose(2)));
    tf::Quaternion q;
    q.setRPY(pose(3), pose(4), pose(5));
    transform.setRotation(q);
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, t, "map", "base_link"));
}

void EncodeRos::publishMap()
{
    sensor_msgs::PointCloud2 map_msg;

    map_builder_.getMap(map_msg);
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = map_frame_;
    map_pub_.publish(map_msg);
}