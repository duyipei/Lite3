#include <hdl_localization/pose_estimator.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <hdl_localization/pose_system.hpp>
#include <hdl_localization/quaternion_state.hpp>
#include <hdl_localization/odom_system.hpp>
#include <kkl/alg/unscented_kalman_filter.hpp>

namespace hdl_localization {

/**
 * @brief constructor
 * @param registration        registration method
 * @param pos                 initial position
 * @param quat                initial orientation
 * @param cool_time_duration  during "cool time", prediction is not performed
 */
PoseEstimator::PoseEstimator(pcl::Registration<PointT, PointT>::Ptr& registration, const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat, double cool_time_duration)
    : cool_time_duration(cool_time_duration), registration(registration){
  last_observation = Eigen::Matrix4f::Identity();
  last_observation.block<3, 3>(0, 0) = quat.toRotationMatrix();
  last_observation.block<3, 1>(0, 3) = pos;

  process_noise = Eigen::MatrixXf::Identity(16, 16);
  process_noise.middleRows(0, 3) *= 1.0;
  process_noise.middleRows(3, 3) *= 1.0;
  process_noise.middleRows(6, 4) *= 0.5;
  process_noise.middleRows(10, 3) *= 1e-6;
  process_noise.middleRows(13, 3) *= 1e-6;

  Eigen::MatrixXf measurement_noise = Eigen::MatrixXf::Identity(7, 7);
  measurement_noise.middleRows(0, 3) *= 0.01;
  measurement_noise.middleRows(3, 4) *= 0.001;

  Eigen::VectorXf mean(16);
  mean.middleRows(0, 3) = pos;
  mean.middleRows(3, 3).setZero();
  mean.middleRows(6, 4) = Eigen::Vector4f(quat.w(), quat.x(), quat.y(), quat.z()).normalized();
  mean.middleRows(10, 3).setZero();
  mean.middleRows(13, 3).setZero();

  Eigen::MatrixXf cov = Eigen::MatrixXf::Identity(16, 16) * 0.01;

  PoseSystem system;
  ukf.reset(new kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>(system, 16, 6, 7, process_noise, measurement_noise, mean, cov));
}

PoseEstimator::~PoseEstimator() {}

/**
 * @brief predict
 * @param stamp    timestamp
 * @param acc      acceleration
 * @param gyro     angular velocity
 */
void PoseEstimator::predict(const rclcpp::Time& stamp) {
  if(!state_valid_) return;
  if (init_stamp.nanoseconds() == 0) {
    init_stamp = stamp;
  }

  if (prev_stamp.nanoseconds() != 0 && stamp <= prev_stamp) return;
  if ((stamp - init_stamp).seconds() < cool_time_duration || prev_stamp.nanoseconds() == 0) {
    prev_stamp = stamp;
    return;
  }

  double dt = (stamp - prev_stamp).seconds();
  prev_stamp = stamp;

  ukf->setProcessNoiseCov(process_noise * dt);
  ukf->system.dt = dt;

  ukf->predict();
  state_valid_ = NormalizeQuaternionState(ukf->mean, ukf->cov, 6);
}

/**
 * @brief predict
 * @param stamp    timestamp
 * @param acc      acceleration
 * @param gyro     angular velocity
 */
void PoseEstimator::predict(const rclcpp::Time& stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro) {
  if(!state_valid_) return;
  if (init_stamp.nanoseconds() == 0) {
    init_stamp = stamp;
  }

  if (prev_stamp.nanoseconds() != 0 && stamp <= prev_stamp) return;
  if ((stamp - init_stamp).seconds() < cool_time_duration || prev_stamp.nanoseconds() == 0) {
    prev_stamp = stamp;
    return;
  }

  double dt = (stamp - prev_stamp).seconds();
  prev_stamp = stamp;

  ukf->setProcessNoiseCov(process_noise * dt);
  ukf->system.dt = dt;

  Eigen::VectorXf control(6);
  control.head<3>() = acc;
  control.tail<3>() = gyro;

  ukf->predict(control);
  state_valid_ = NormalizeQuaternionState(ukf->mean, ukf->cov, 6);
}

/**
 * @brief update the state of the odomety-based pose estimation
 */
void PoseEstimator::predict_odom(const Eigen::Matrix4f& odom_delta) {
  if(!state_valid_) return;
  if(!odom_ukf) {
    Eigen::MatrixXf odom_process_noise = Eigen::MatrixXf::Identity(7, 7);
    Eigen::MatrixXf odom_measurement_noise = Eigen::MatrixXf::Identity(7, 7) * 1e-3;

    Eigen::VectorXf odom_mean(7);
    odom_mean.block<3, 1>(0, 0) = Eigen::Vector3f(ukf->mean[0], ukf->mean[1], ukf->mean[2]);
    odom_mean.block<4, 1>(3, 0) = Eigen::Vector4f(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]);
    Eigen::MatrixXf odom_cov = Eigen::MatrixXf::Identity(7, 7) * 1e-2;

    OdomSystem odom_system;
    odom_ukf.reset(new kkl::alg::UnscentedKalmanFilterX<float, OdomSystem>(odom_system, 7, 7, 7, odom_process_noise, odom_measurement_noise, odom_mean, odom_cov));
  }

  // invert quaternion if the rotation axis is flipped
  Eigen::Quaternionf quat(odom_delta.block<3, 3>(0, 0));
  if(odom_quat().coeffs().dot(quat.coeffs()) < 0.0) {
    quat.coeffs() *= -1.0f;
  }

  Eigen::VectorXf control(7);
  control.middleRows(0, 3) = odom_delta.block<3, 1>(0, 3);
  control.middleRows(3, 4) = Eigen::Vector4f(quat.w(), quat.x(), quat.y(), quat.z());

  Eigen::MatrixXf process_noise = Eigen::MatrixXf::Identity(7, 7);
  process_noise.topLeftCorner(3, 3) = Eigen::Matrix3f::Identity() * odom_delta.block<3, 1>(0, 3).norm() + Eigen::Matrix3f::Identity() * 1e-3;
  process_noise.bottomRightCorner(4, 4) = Eigen::Matrix4f::Identity() * (1 - std::abs(quat.w())) + Eigen::Matrix4f::Identity() * 1e-3;

  odom_ukf->setProcessNoiseCov(process_noise);
  odom_ukf->predict(control);
  state_valid_ = state_valid_ && NormalizeQuaternionState(odom_ukf->mean, odom_ukf->cov, 3);
}

/**
 * @brief correct
 * @param cloud   input cloud
 * @return cloud aligned to the globalmap
 */
bool PoseEstimator::predict_odom_at(const rclcpp::Time& stamp, const Eigen::Matrix4f& leg_pose) {
  if(!state_valid_ || !leg_pose.allFinite() || stamp.nanoseconds() <= 0 ||
     (leg_prediction_valid_ && stamp <= leg_prediction_stamp_)) return false;
  if(leg_prediction_valid_) predict_odom(leg_prediction_pose_.inverse()*leg_pose);
  if(!state_valid_) return false;
  leg_prediction_pose_=leg_pose;
  leg_prediction_stamp_=stamp;
  leg_prediction_valid_=true;
  return true;
}

pcl::PointCloud<PoseEstimator::PointT>::Ptr PoseEstimator::correct(const rclcpp::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud, const PoseRefiner& refiner, const PoseValidator& validator, const Eigen::Matrix4f* registration_guess, bool initialize_pose) {
  if (init_stamp.nanoseconds() == 0) {
    init_stamp = stamp;
  }

  if(!state_valid_) return nullptr;
  Eigen::Matrix4f no_guess = last_observation;
  Eigen::Matrix4f imu_guess;
  Eigen::Matrix4f odom_guess;
  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

  if(!odom_ukf) {
    init_guess = imu_guess = matrix();
  } else {
    imu_guess = matrix();
    odom_guess = odom_matrix();

    Eigen::VectorXf imu_mean(7);
    Eigen::MatrixXf imu_cov = Eigen::MatrixXf::Identity(7, 7);
    imu_mean.block<3, 1>(0, 0) = ukf->mean.block<3, 1>(0, 0);
    imu_mean.block<4, 1>(3, 0) = ukf->mean.block<4, 1>(6, 0);

    imu_cov.block<3, 3>(0, 0) = ukf->cov.block<3, 3>(0, 0);
    imu_cov.block<3, 4>(0, 3) = ukf->cov.block<3, 4>(0, 6);
    imu_cov.block<4, 3>(3, 0) = ukf->cov.block<4, 3>(6, 0);
    imu_cov.block<4, 4>(3, 3) = ukf->cov.block<4, 4>(6, 6);

    Eigen::VectorXf odom_mean = odom_ukf->mean;
    Eigen::MatrixXf odom_cov = odom_ukf->cov;

    AlignQuaternionHemisphere(odom_mean, odom_cov, 3, imu_mean.tail<4>());

    Eigen::MatrixXf inv_imu_cov = imu_cov.inverse();
    Eigen::MatrixXf inv_odom_cov = odom_cov.inverse();

    Eigen::MatrixXf fused_cov = (inv_imu_cov + inv_odom_cov).inverse();
    Eigen::VectorXf fused_mean = fused_cov * inv_imu_cov * imu_mean + fused_cov * inv_odom_cov * odom_mean;

    init_guess.block<3, 1>(0, 3) = Eigen::Vector3f(fused_mean[0], fused_mean[1], fused_mean[2]);
    init_guess.block<3, 3>(0, 0) = Eigen::Quaternionf(fused_mean[3], fused_mean[4], fused_mean[5], fused_mean[6]).normalized().toRotationMatrix();
  }

  pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
  if(registration_guess) {
    if(!registration_guess->allFinite()) return nullptr;
    init_guess=*registration_guess;
  }
  registration->setInputSource(cloud);
  registration->align(*aligned, init_guess);

  Eigen::Matrix4f trans = registration->getFinalTransformation();
  if(!registration->hasConverged() || !trans.allFinite()) return nullptr;
  if(refiner) {
    Eigen::Matrix4f refined;
    if(!refiner(trans, refined) || !refined.allFinite()) return nullptr;
    trans = refined;
    pcl::transformPointCloud(*cloud, *aligned, trans);
  }
  Eigen::Vector3f p = trans.block<3, 1>(0, 3);
  Eigen::Quaternionf q(trans.block<3, 3>(0, 0));

  if(quat().coeffs().dot(q.coeffs()) < 0.0f) {
    q.coeffs() *= -1.0f;
  }

  Eigen::VectorXf observation(7);
  observation.middleRows(0, 3) = p;
  observation.middleRows(3, 4) = Eigen::Vector4f(q.w(), q.x(), q.y(), q.z());
  // Commit accepted observation, diagnostic errors and stamp atomically only
  // after both estimator corrections remain valid.
  const Eigen::VectorXf saved_mean=ukf->mean;
  const Eigen::MatrixXf saved_cov=ukf->cov;
  Eigen::VectorXf saved_odom_mean;
  Eigen::MatrixXf saved_odom_cov;
  if(odom_ukf) {saved_odom_mean=odom_ukf->mean;saved_odom_cov=odom_ukf->cov;}
  ukf->correct(observation);
  bool valid=NormalizeQuaternionState(ukf->mean,ukf->cov,6);
  if(valid && odom_ukf) {
    AlignQuaternionHemisphere(odom_ukf->mean,odom_ukf->cov,3,observation.tail<4>());
    odom_ukf->correct(observation);
    valid=NormalizeQuaternionState(odom_ukf->mean,odom_ukf->cov,3);
  }
  if(valid && initialize_pose) {
    // Acquisition establishes the first accepted metric pose. A provisional
    // search must not repeatedly average against the unchanged coarse prior:
    // doing so can stabilize halfway between a correct match and the seed.
    // Both filters receive the SAME observation, and are still rolled back
    // unless the external multi-scan acquisition validator accepts it.
    ukf->mean.head<3>()=p;
    ukf->mean.segment<4>(6)=observation.tail<4>();
    valid=NormalizeQuaternionState(ukf->mean,ukf->cov,6);
    if(valid && odom_ukf) {
      odom_ukf->mean=observation;
      valid=NormalizeQuaternionState(odom_ukf->mean,odom_ukf->cov,3);
    }
  }
  // Outer geometric/fitness acceptance is part of this transaction, before
  // any accepted observation, diagnostic error or correction time is committed.
  if(valid && validator) valid=validator(matrix());
  if(!valid) {
    ukf->mean=saved_mean;ukf->cov=saved_cov;
    if(odom_ukf) {odom_ukf->mean=saved_odom_mean;odom_ukf->cov=saved_odom_cov;}
    return nullptr;
  }
  last_correction_stamp=stamp;
  last_observation=trans;
  wo_pred_error=no_guess.inverse()*trans;
  if(refiner) {
    static size_t diagnostic_count = 0;
    if(++diagnostic_count % 20 == 0) {
      const auto post = quat();
      RCLCPP_INFO(rclcpp::get_logger("fine_pose_diagnostic"),
        "measurement_q=%.6f,%.6f,%.6f,%.6f posterior_q=%.6f,%.6f,%.6f,%.6f state_q_norm=%.6f",
        q.w(),q.x(),q.y(),q.z(),post.w(),post.x(),post.y(),post.z(),ukf->mean.middleRows(6,4).norm());
    }
  }
  imu_pred_error = imu_guess.inverse() * trans;

  if(odom_ukf) odom_pred_error = odom_guess.inverse() * trans;

  return aligned;
}

/* getters */
rclcpp::Time PoseEstimator::last_correction_time() const {
  return last_correction_stamp;
}

Eigen::Vector3f PoseEstimator::pos() const {
  return Eigen::Vector3f(ukf->mean[0], ukf->mean[1], ukf->mean[2]);
}

Eigen::Vector3f PoseEstimator::vel() const {
  return Eigen::Vector3f(ukf->mean[3], ukf->mean[4], ukf->mean[5]);
}

Eigen::Quaternionf PoseEstimator::quat() const {
  return Eigen::Quaternionf(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]).normalized();
}

Eigen::Matrix4f PoseEstimator::matrix() const {
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3, 3>(0, 0) = quat().toRotationMatrix();
  m.block<3, 1>(0, 3) = pos();
  return m;
}

Eigen::Vector3f PoseEstimator::odom_pos() const {
  return Eigen::Vector3f(odom_ukf->mean[0], odom_ukf->mean[1], odom_ukf->mean[2]);
}

Eigen::Quaternionf PoseEstimator::odom_quat() const {
  return Eigen::Quaternionf(odom_ukf->mean[3], odom_ukf->mean[4], odom_ukf->mean[5], odom_ukf->mean[6]).normalized();
}

Eigen::Matrix4f PoseEstimator::odom_matrix() const {
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3, 3>(0, 0) = odom_quat().toRotationMatrix();
  m.block<3, 1>(0, 3) = odom_pos();
  return m;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::wo_prediction_error() const {
  return wo_pred_error;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::imu_prediction_error() const {
  return imu_pred_error;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::odom_prediction_error() const {
  return odom_pred_error;
}
}
