#include "map_builder.h"

namespace encode
{

MapBuilder::MapBuilder() :
    pose_(Eigen::Matrix4f::Identity()), odometry_pose_(Eigen::Matrix4f::Identity()),
    sequence_count_(0), map_update_count_(200)
{
}

KeyFrame::Ptr MapBuilder::addVertex(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point_cloud, Eigen::Matrix4f& pose)
{
    static int vertex_count = 0;

    KeyFrame::Ptr key_frame(new KeyFrame());
    key_frame->setId(vertex_count);
    key_frame->setPose(pose);
    key_frames_.push_back(key_frame);
    vertex_count++;

    return key_frame;
}

void MapBuilder::saveOdometry()
{
    std::ofstream oStream;
    oStream.open("encode.txt");
    for(const KeyFrame::Ptr& frame : key_frames_) {
        Eigen::Matrix4f pose = frame->getPose();
        oStream << pose(0, 0) << ' ' << pose(0, 1) << ' '
                << pose(0, 2) << ' ' << pose(0, 3) << ' ';
        oStream << pose(1, 0) << ' ' << pose(1, 1) << ' '
                << pose(1, 2) << ' ' << pose(1, 3) << ' ';
        oStream << pose(2, 0) << ' ' << pose(2, 1) << ' '
                << pose(2, 2) << ' ' << pose(2, 3);
        oStream << std::endl;
    }
    std::cout << "Odometry pose saved!" << std::endl;
}

void MapBuilder::addPointCloud_odometry_only(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point_cloud, Transform& delta_pose)
{
    static int scan_count = 0;
    scan_count++;
    odometry_pose_ = odometry_pose_ * delta_pose.matrix();

    KeyFrame::Ptr frame = addVertex(point_cloud, odometry_pose_);
    if ((scan_count % 2) == 0) {
        pose_ = odometry_pose_;
        // downsample the pointcloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        CloudDownSample sampler(0.5);
        sampler.inputCloud(*point_cloud);
        sampler.getDownSampledCloud(*sampled_cloud);

        // update map
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*sampled_cloud, *transformed_cloud, pose_);
        std::unique_lock<std::mutex> locker(map_mutex_);
        map_ += *transformed_cloud;

        sequence_count_++;
        if ((sequence_count_ % map_update_count_) == 0) {
            downSampleMap();
        }
    }
}

void MapBuilder::downSampleMap() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    CloudDownSample sampler(0.5);
    sampler.inputCloud(map_);
    sampler.getDownSampledCloud(*sampled_cloud);
    map_ = *sampled_cloud;
}

CloudDownSample::CloudDownSample(double resolution) {
  mResolution = resolution;
}
CloudDownSample::~CloudDownSample() {}

void CloudDownSample::inputCloud(const pcl::PointCloud<pcl::PointXYZI> &cloud) {
  pointAttr attr;
  for (size_t i = 0; i < cloud.points.size(); i++) {
    double scaleX =
        static_cast<int>(cloud.points[i].x / mResolution) * mResolution;
    double scaleY =
        static_cast<int>(cloud.points[i].y / mResolution) * mResolution;
    double scaleZ =
        static_cast<int>(cloud.points[i].z / mResolution) * mResolution;
    if (mMapGrid.find(scaleX) != mMapGrid.end() &&
        mMapGrid[scaleX].find(scaleY) != mMapGrid[scaleX].end() &&
        mMapGrid[scaleX][scaleY].find(scaleZ) !=
            mMapGrid[scaleX][scaleY].end()) {
      attr = mMapGrid[scaleX][scaleY][scaleZ];
      attr.num++;
      attr.x = attr.x + (cloud.points[i].x - attr.x) / attr.num;
      attr.y = attr.y + (cloud.points[i].y - attr.y) / attr.num;
      attr.z = attr.z + (cloud.points[i].z - attr.z) / attr.num;
      attr.intensity = attr.intensity +
                       (cloud.points[i].intensity - attr.intensity) / attr.num;
      mMapGrid[scaleX][scaleY][scaleZ] = attr;
    } else {
      attr.x = cloud.points[i].x;
      attr.y = cloud.points[i].y;
      attr.z = cloud.points[i].z;
      attr.intensity = cloud.points[i].intensity;
      attr.num = 1;
      mMapGrid[scaleX][scaleY][scaleZ] = attr;
    }
  }
}

void CloudDownSample::getDownSampledCloud(
    pcl::PointCloud<pcl::PointXYZI> &downCloud) {
  downCloud.clear();
  pcl::PointXYZI point;
  for (auto &mapX : mMapGrid) {
    for (auto &mapY : mapX.second) {
      for (auto &mapZ : mapY.second) {
        point.x = mapZ.second.x;
        point.y = mapZ.second.y;
        point.z = mapZ.second.z;
        point.intensity = mapZ.second.intensity;
        downCloud.push_back(point);
      }
    }
  }
}

void CloudDownSample::clear() { mMapGrid.clear(); }

} // namespace lidar_slam_3d
