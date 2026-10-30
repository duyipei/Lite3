#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <memory>
#include <functional>
#include <boost/optional.hpp>

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>

namespace kkl {
  namespace alg {
template<typename T, class System> class UnscentedKalmanFilterX;
  }
}

namespace hdl_localization {

class PoseSystem;
class OdomSystem;

/**
 * @brief scan matching-based pose estimator
 */
class PoseEstimator {
public:
  using PointT = pcl::PointXYZI;

  /**
   * @brief constructor
   * @param registration        registration method
   * @param pos                 initial position
   * @param quat                initial orientation
   * @param cool_time_duration  during "cool time", prediction is not performed
   */
  PoseEstimator(pcl::Registration<PointT, PointT>::Ptr& registration, const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat, double cool_time_duration = 1.0);
  ~PoseEstimator();

  /**
   * @brief predict
   * @param stamp    timestamp
   */
  void predict(const rclcpp::Time& stamp);

  /**
   * @brief predict
   * @param stamp    timestamp
   * @param acc      acceleration
   * @param gyro     angular velocity
   */
  void predict(const rclcpp::Time& stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro);

  /**
   * @brief update the state of the odomety-based pose estimation
   */
  void predict_odom(const Eigen::Matrix4f& odom_delta);
  // Absolute, same-scan leg poses prevent reapplying cumulative motion after
  // a rejected registration. The prediction cursor advances independently.
  bool predict_odom_at(const rclcpp::Time& stamp, const Eigen::Matrix4f& leg_pose);

  /**
   * @brief correct
   * @param cloud   input cloud
   * @return cloud aligned to the globalmap
   */
  using PoseRefiner = std::function<bool(const Eigen::Matrix4f&, Eigen::Matrix4f&)>;
  using PoseValidator = std::function<bool(const Eigen::Matrix4f&)>;
  // Refine the registration observation BEFORE either UKF consumes it.
  // False rejects the observation without changing the accepted correction stamp.
  pcl::PointCloud<PointT>::Ptr correct(const rclcpp::Time& stamp,
      const pcl::PointCloud<PointT>::ConstPtr& cloud,
      const PoseRefiner& refiner = PoseRefiner(),
      const PoseValidator& validator = PoseValidator(),
      const Eigen::Matrix4f* registration_guess = nullptr,
      bool initialize_pose = false);

  /* getters */
  rclcpp::Time last_correction_time() const;

  Eigen::Vector3f pos() const;
  Eigen::Vector3f vel() const;
  Eigen::Quaternionf quat() const;
  Eigen::Matrix4f matrix() const;

  Eigen::Vector3f odom_pos() const;
  Eigen::Quaternionf odom_quat() const;
  Eigen::Matrix4f odom_matrix() const;

  const boost::optional<Eigen::Matrix4f>& wo_prediction_error() const;
  const boost::optional<Eigen::Matrix4f>& imu_prediction_error() const;
  const boost::optional<Eigen::Matrix4f>& odom_prediction_error() const;

private:
  rclcpp::Time init_stamp;             // when the estimator was initialized
  rclcpp::Time prev_stamp;             // when the estimator was updated last time
  rclcpp::Time last_correction_stamp;  // when the estimator performed the correction step
  double cool_time_duration;        //

  bool state_valid_ = true;
  bool leg_prediction_valid_ = false;
  rclcpp::Time leg_prediction_stamp_;
  Eigen::Matrix4f leg_prediction_pose_ = Eigen::Matrix4f::Identity();
  Eigen::MatrixXf process_noise;
  std::unique_ptr<kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>> ukf;
  std::unique_ptr<kkl::alg::UnscentedKalmanFilterX<float, OdomSystem>> odom_ukf;

  Eigen::Matrix4f last_observation;
  boost::optional<Eigen::Matrix4f> wo_pred_error;
  boost::optional<Eigen::Matrix4f> imu_pred_error;
  boost::optional<Eigen::Matrix4f> odom_pred_error;

  pcl::Registration<PointT, PointT>::Ptr registration;
  };

}  // namespace hdl_localization

#endif  // POSE_ESTIMATOR_HPP
