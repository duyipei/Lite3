#pragma once
#include <Eigen/Dense>
#include <cmath>

namespace hdl_localization {
// Project an additive quaternion state and ALL cross-covariances together.
// Radial variance is numerical regularization for LLT, not attitude noise.
inline bool NormalizeQuaternionState(Eigen::VectorXf& mean,
                                     Eigen::MatrixXf& cov, int offset) {
  if(offset < 0 || mean.size() < offset+4 || cov.rows()!=mean.size() ||
     cov.cols()!=mean.size() || !mean.allFinite() || !cov.allFinite()) return false;
  const Eigen::Vector4f raw=mean.segment<4>(offset);
  const float norm=raw.norm();
  if(!std::isfinite(norm) || norm<1e-6f) return false;
  const Eigen::Vector4f q=raw/norm;
  Eigen::MatrixXf jacobian=Eigen::MatrixXf::Identity(mean.size(),mean.size());
  jacobian.block<4,4>(offset,offset)=(Eigen::Matrix4f::Identity()-q*q.transpose())/norm;
  Eigen::MatrixXf projected=jacobian*cov*jacobian.transpose();
  projected.block<4,4>(offset,offset)+=1e-6f*q*q.transpose();
  projected=((projected+projected.transpose())*.5f).eval();
  if(!projected.allFinite() || Eigen::LLT<Eigen::MatrixXf>(projected).info()!=Eigen::Success) return false;
  mean.segment<4>(offset)=q;
  cov=projected;
  return true;
}
// q and -q describe the same rotation. A sign change is a coordinate change
// for the covariance too, including position/orientation cross terms.
inline void AlignQuaternionHemisphere(Eigen::VectorXf& mean,
                                      Eigen::MatrixXf& cov, int offset,
                                      const Eigen::Vector4f& reference) {
  if(reference.dot(mean.segment<4>(offset)) < 0) {
    mean.segment<4>(offset)*=-1;
    cov.middleRows(offset,4)*=-1;
    cov.middleCols(offset,4)*=-1;
  }
}
}
