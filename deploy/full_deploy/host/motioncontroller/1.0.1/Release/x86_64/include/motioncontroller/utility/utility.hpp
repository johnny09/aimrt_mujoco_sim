#pragma once
#include "motioncontroller/function_type.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <kdl/jntarray.hpp>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
namespace mc {
namespace utility {
inline std::vector<std::string> GetStringSplit(std::string str,
                                               const char split) {
  std::istringstream iss(str); // input stream
  std::string sub_str;         // recv buffer
  std::vector<std::string> vec_str;
  vec_str.clear();
  while (getline(iss, sub_str, split)) // seperate using split
  {
    vec_str.push_back(sub_str); // output
  }
  return vec_str;
}

template <class StType>
inline type::FunctionReturnType bin_to_st(void *bin, StType &st) {
  if (bin != nullptr) {
    memcpy((void *)(&st), (void *)(bin), sizeof(StType));
    return type::FunctionReturnType::kDone;
  } else
    return type::FunctionReturnType::kError;
}
template <class StType>
inline type::FunctionReturnType st_to_bin(const StType &st, void *bin) {
  if (bin != nullptr) {
    memcpy((void *)(bin), (void *)&st, sizeof(StType));
    return type::FunctionReturnType::kDone;
  } else
    return type::FunctionReturnType::kError;
}

inline constexpr double deg2rad(double degrees) {
  return degrees * M_PI / 180.0;
}

inline constexpr double rad2deg(double radians) {
  return radians * 180.0 / M_PI;
}

inline bool isAllZero(const std::vector<double> &vec,
                      double tolerance = 1e-10) {
  return std::all_of(vec.begin(), vec.end(),
                     [tolerance](double x) { return std::abs(x) < tolerance; });
}

inline std::vector<double> KDLJntArraytoStdVector(const KDL::JntArray &joints) {
  std::vector<double> result(joints.rows());
  for (unsigned int i = 0; i < joints.rows(); i++) {
    result[i] = joints(i);
  }
  return result;
}

inline Eigen::VectorXd KDLJntArrayToEigenVector(const KDL::JntArray &joints) {
  Eigen::VectorXd vec(joints.rows());
  for (unsigned int i = 0; i < joints.rows(); ++i) {
    vec(i) = joints(i);
  }
  return vec;
}

inline KDL::JntArray EigenVectorToKDLJntArray(const Eigen::VectorXd &vec) {
  KDL::JntArray joints(vec.size());
  for (unsigned int i = 0; i < vec.size(); ++i) {
    joints(i) = vec(i);
  }
  return joints;
}

inline KDL::Jacobian EigenMatToKDLJacobian(const Eigen::MatrixXd &mat) {
  KDL::Jacobian jac(mat.cols());
  for (int col = 0; col < mat.cols(); ++col) {
    for (int row = 0; row < std::min<int>(6, mat.rows()); ++row) {
      jac(row, col) = mat(row, col);
    }
  }
  return jac;
}

inline Eigen::Matrix<double, 6, 1>
KDLTwistToEigenVector(const KDL::Twist &twist) {
  Eigen::Matrix<double, 6, 1> vec;
  vec.head<3>() = Eigen::Vector3d(twist.vel.data);
  vec.tail<3>() = Eigen::Vector3d(twist.rot.data);
  return vec;
}

inline KDL::JntArray StdVectortoKDLJntArray(const std::vector<double> &joints) {
  KDL::JntArray result(joints.size());
  for (size_t i = 0; i < joints.size(); i++) {
    result(i) = joints[i];
  }
  return result;
}
inline Eigen::VectorXd StdVectorToEigenVector(const std::vector<double> &vec) {
  Eigen::VectorXd eigen_vec(vec.size());
  for (size_t i = 0; i < vec.size(); ++i) {
    eigen_vec(i) = vec[i];
  }
  return eigen_vec;
}

inline Eigen::Vector3d KDLVectorToEigen(const KDL::Vector &kdl_vec) {
  return Eigen::Vector3d(kdl_vec[0], kdl_vec[1], kdl_vec[2]);
}

inline Eigen::Matrix4d KdlFrameToEigenMatrix4d(const KDL::Frame &frame) {
  Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();

  // 直接映射旋转部分
  Eigen::Map<const Eigen::Matrix3d> rot_mat(frame.M.data);
  mat.block<3, 3>(0, 0) = rot_mat;

  // 设置平移部分
  mat.block<3, 1>(0, 3) = Eigen::Vector3d(frame.p.data);

  return mat;
}

inline KDL::Frame EigenMatrix4dToKdlFrame(const Eigen::Matrix4d &mat) {
  // 提取旋转部分
  KDL::Rotation rot;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      rot(i, j) = mat(i, j);
    }
  }

  // 提取平移部分
  KDL::Vector pos(mat(0, 3), mat(1, 3), mat(2, 3));

  return KDL::Frame(rot, pos);
}

inline Eigen::Isometry3d KDLFrameToEigenIsometry3d(const KDL::Frame &frame) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  // 映射 KDL::Rotation 到 Eigen::Matrix3d
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      T.linear()(i, j) = frame.M(i, j);

  // 映射 KDL::Vector 到 Eigen::Vector3d
  T.translation() = Eigen::Map<const Eigen::Vector3d>(frame.p.data);

  return T;
}

inline KDL::Frame EigenIsometry3dToKDLFrame(const Eigen::Isometry3d &T) {
  KDL::Frame frame;

  // 映射 Eigen::Matrix3d 到 KDL::Rotation
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      frame.M(i, j) = T.linear()(i, j);

  // 映射 Eigen::Vector3d 到 KDL::Vector
  frame.p[0] = T.translation()(0);
  frame.p[1] = T.translation()(1);
  frame.p[2] = T.translation()(2);

  return frame;
}

} // namespace utility
} // namespace mc