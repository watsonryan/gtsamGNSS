#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

namespace gtsam {

using PhaseBias = Vector1;

struct GaussianMixtureComponent {
  int total_obs{0};
  int component_obs{0};
  double weight{0.0};
  Eigen::RowVectorXd mean;
  Eigen::MatrixXd cov;
};

} // namespace gtsam
