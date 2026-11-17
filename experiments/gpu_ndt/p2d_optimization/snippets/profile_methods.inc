  double linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double,6,6>* H,
                   Eigen::Matrix<double,6,1>* b) override {
    ++profile_->p2d_linearizations;
    return Base::linearize(trans,H,b);
  }
  double compute_error(const Eigen::Isometry3d& trans) override {
    ++profile_->p2d_error_evaluations;
    const auto begin = Clock::now();
    const double error = Base::compute_error(trans);
    profile_->p2d_error_evaluation_ms += elapsed_ms(begin);
    if (!std::isfinite(error)) throw std::runtime_error("nonfinite P2D candidate error");
    if (profile_->p2d_verify_costs) {
      // Same fixed correspondences and scalar objective, but the original
      // full derivative path. Verification mode ONLY, excluded from timing.
      Eigen::Matrix<double,6,6> H;
      Eigen::Matrix<double,6,1> b;
      const double reference = this->ndt_cuda_->compute_error(trans,&H,&b);
      if (!std::isfinite(reference) || !H.allFinite() || !b.allFinite())
        throw std::runtime_error("nonfinite P2D reference derivatives");
      const double difference = std::abs(error-reference);
      const double scale = std::max(1.0,std::abs(reference));
      profile_->p2d_max_abs_error = std::max(profile_->p2d_max_abs_error,difference);
      profile_->p2d_max_relative_error = std::max(profile_->p2d_max_relative_error,difference/scale);
      if (difference > 2e-5 + 2e-6*scale)
        throw std::runtime_error("P2D scalar/reference mismatch; stop before workspace/performance stage");
    }
    return error;
  }
