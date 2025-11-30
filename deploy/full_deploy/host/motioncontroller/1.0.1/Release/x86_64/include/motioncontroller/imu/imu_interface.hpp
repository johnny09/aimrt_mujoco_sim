#pragma once
#include <Eigen/Dense>
namespace mc {
namespace imu {
class IMUInterface {
public:
  // Initialize the IMU interface.
  // Returns true if initialization is successful, false otherwise.
  virtual bool Init() = 0;

  // Read IMU data.
  // Returns true if data is successfully read, false otherwise.
  // The IMU data is stored in the euler_angle_, angular_velocity_, and
  // linear_acceleration_ member variables. The euler_angle_ vector represents
  // the Euler angle vector in the world's NWU coordinate system. The
  // angular_velocity_ vector represents the angular velocity vector in the
  // sensor's local coordinate system. The linear_acceleration_ vector
  // represents the linear acceleration vector in the sensor's local coordinate
  // system. The units of the euler angle values are in radians. The units of
  // the angular velocity values are typically in radians per second (rad/s).
  // The units of the linear acceleration values are typically in meters per
  // second squared (m/s^2).
  virtual bool ReadIMUData() = 0;

  // Returns the Euler angle vector.
  // The Euler angle vector represents the orientation of the sensor in the
  // world's NWU coordinate system. The returned vector is the euler angle
  // represented in the order of Z-Y-X, which corresponds to the yaw, pitch, and
  // roll angles, respectively. The units of the euler angle values are in
  // radians.
  Eigen::Vector3d GetEulerAngle() { return euler_angle_; }

  // Returns the orientation of the sensor as a quaternion.
  // The quaternion represents the orientation of the sensor in the world's NWU
  // coordinate system. The quaternion is derived from the Euler angle vector
  // using the Z-Y-X (yaw-pitch-roll) convention. The returned quaternion is in
  // the form of (w, x, y, z), where w is the scalar part and (x, y, z) is the
  // vector part.
  Eigen::Quaterniond GetQuaternion() {
    // ZYX order: yaw (Z), pitch (Y), roll (X)
    Eigen::AngleAxisd yawAngle(euler_angle_[0], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(euler_angle_[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollAngle(euler_angle_[2], Eigen::Vector3d::UnitX());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
  }

  // Returns the angular velocity vector.
  // The angular velocity vector represents the angular velocity of the sensor
  // in the sensor's local coordinate system. The x, y, and z components of the
  // vector correspond to the angular velocity around the x, y, and z axes,
  // respectively. The units of the angular velocity values are typically in
  // radians per second (rad/s).
  Eigen::Vector3d GetAngularVelocity() { return angular_velocity_; }
  // Returns the linear acceleration vector.
  // The linear acceleration vector represents the acceleration experienced by
  // the sensor in the sensor's local coordinate system. The x, y, and z
  // components of the vector correspond to the acceleration along the x, y, and
  // z axes, respectively. The units of the acceleration values are typically in
  // meters per second squared (m/s^2).
  Eigen::Vector3d GetLinearAcceleration() { return linear_acceleration_; }

  // Returns the gravity projection vector.
  // The gravity projection vector represents the projection of the gravity
  // vector onto the sensor's local coordinate system. The x, y, and z
  // components of the vector correspond to the projection along the x, y, and z
  // axes, respectively. The range of the values of the components of the vector
  // is between -1 and 1.
  Eigen::Vector3d GetGravityProjection() {
    Eigen::Matrix3d rotm;
    Eigen::Vector3d euler_angle_with_zero_yaw = euler_angle_;
    euler_angle_with_zero_yaw[0] = 0;
    rotm = euler_zyx_To_rot_matrix(euler_angle_with_zero_yaw);
    return rotm.col(2);
  }

private:
  Eigen::Matrix3d euler_zyx_To_rot_matrix(const Eigen::Vector3d &euler_angle) {
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitX());
    return R;
  }

protected:
  Eigen::Vector3d euler_angle_;         // yaw,pitch,roll
  Eigen::Vector3d angular_velocity_;    // roll,pitch,yaw
  Eigen::Vector3d linear_acceleration_; // x,y,z
};
} // namespace imu
} // namespace mc