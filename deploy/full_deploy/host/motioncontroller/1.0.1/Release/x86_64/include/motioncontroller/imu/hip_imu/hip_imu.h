#include "Eigen/Dense"
#include "motioncontroller/imu/hip_imu/hipnuc_dec.h"
#include "motioncontroller/imu/imu_interface.hpp"

namespace mc {
namespace imu {
class HipIMU : public IMUInterface {
public:
  HipIMU(const char *port_name, int baud_rate);
  ~HipIMU();

  bool Init() override;
  bool ReadIMUData() override;

private:
  const char *port_name_;
  int baud_rate_;
  uint8_t recv_buf_[2048];
  int fd_ = -1;
  hipnuc_raw_t hipnuc_raw_ = {0};
  Eigen::Matrix3d rot_ = Eigen::Matrix3d::Zero();
  Eigen::Vector3d angular_velocity_tmp_ = Eigen::VectorXd::Zero(3);
  Eigen::Vector3d linear_acceleration_tmp_ = Eigen::VectorXd::Zero(3);
};

} // namespace imu
} // namespace mc
