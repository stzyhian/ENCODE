#ifndef __TRANSFORM__H
#define __TRANSFORM__H

#include <Eigen/Geometry>

namespace encode {

typedef Eigen::Matrix<double, 6, 1> Vector6;
typedef Eigen::Quaterniond Rotation;
typedef Eigen::Vector3d Translation;
typedef Eigen::Matrix<double, 4, 4> Matrix;

class Transform {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Transform() {
    rotation_.setIdentity();
    translation_.setZero();
  }

  Transform(const Translation& translation, const Rotation& rotation)
      : translation_(translation), rotation_(rotation) {}

  Transform(Matrix matrix)
      : translation_(matrix.topRightCorner<3, 1>()),
        rotation_(matrix.topLeftCorner<3, 3>()) {}

  Transform& operator=(const Transform& trans) {
    rotation_.x() = trans.rotation().x();
    rotation_.y() = trans.rotation().y();
    rotation_.z() = trans.rotation().z();
    rotation_.w() = trans.rotation().w();
    translation_(0) = trans.translation()(0);
    translation_(1) = trans.translation()(1);
    translation_(2) = trans.translation()(2);
  }

  Transform(const Transform& trans) {
    rotation_.x() = trans.rotation().x();
    rotation_.y() = trans.rotation().y();
    rotation_.z() = trans.rotation().z();
    rotation_.w() = trans.rotation().w();
    translation_(0) = trans.translation()(0);
    translation_(1) = trans.translation()(1);
    translation_(2) = trans.translation()(2);
  }

  Transform(const Transform&& trans) {
    rotation_.x() = trans.rotation().x();
    rotation_.y() = trans.rotation().y();
    rotation_.z() = trans.rotation().z();
    rotation_.w() = trans.rotation().w();
    translation_(0) = trans.translation()(0);
    translation_(1) = trans.translation()(1);
    translation_(2) = trans.translation()(2);
  }

  const Rotation& rotation() const { return rotation_; }

  const Translation& translation() const { return translation_; }

  Eigen::Matrix4f matrix() const {
    Eigen::Matrix4f matrix;
    Eigen::Quaternionf rr = rotation_.cast<float>();
    Eigen::Vector3f tt = translation_.cast<float>();
    matrix.setIdentity();
    matrix.topLeftCorner<3, 3>() = rr.matrix();
    matrix.topRightCorner<3, 1>() = tt;
    return matrix;
  }

  Transform inverse() const {
    const Rotation rotation_inverted(rotation_.w(), -rotation_.x(),
                                     -rotation_.y(), -rotation_.z());
    return Transform(-(rotation_inverted * translation_), rotation_inverted);
  }

  Transform operator*(const Transform& rhs) const {
    return Transform(translation_ + rotation_ * rhs.translation(),
                     rotation_ * rhs.rotation());
  }

  static Transform exp(const Vector6& vector) {
    constexpr double kEpsilon = 1e-8;
    const double norm = vector.tail<3>().norm();
    if (norm < kEpsilon) {
      return Transform(vector.head<3>(), Rotation::Identity());
    } else {
      return Transform(vector.head<3>(), Rotation(Eigen::AngleAxisd(
                                             norm, vector.tail<3>() / norm)));
    }
  }

  Vector6 log() const {
    Eigen::AngleAxisd angle_axis(rotation_);
    return (Vector6() << translation_, angle_axis.angle() * angle_axis.axis())
        .finished();
  }

 private:
  Rotation rotation_;
  Translation translation_;
};
}  // namespace encode

#endif  //__TRANSFORM__H