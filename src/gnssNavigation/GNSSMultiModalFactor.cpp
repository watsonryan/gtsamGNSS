/**
 *  @file   GNSSMultiModalFactor.cpp
 *  @author Watson
 *  @brief  GNSS factor with a Gaussian-mixture residual model
 **/

#include <gtsam/gnssNavigation/GNSSMultiModalFactor.h>
#include <gtsam/gnssNavigation/GnssErrors.h>

#include <cmath>
#include <Eigen/Cholesky>
#include <iostream>
#include <limits>
#include <string>

namespace gtsam {
namespace {
constexpr double kPi = 3.14159265358979323846;
}

void GNSSMultiModalFactor::validateGmmOrThrow(
    const std::vector<GaussianMixtureComponent>& gmm) {
  if (gmm.empty()) {
    throw GnssInvalidModelError("Gaussian mixture model is empty");
  }
  for (size_t i = 0; i < gmm.size(); ++i) {
    if (gmm[i].mean.size() != 2 || gmm[i].cov.rows() != 2 ||
        gmm[i].cov.cols() != 2) {
      throw GnssInvalidModelError(
          "Gaussian mixture component " + std::to_string(i) +
          " must have mean size 2 and covariance shape 2x2");
    }
  }
}

std::pair<Vector, Matrix2> GNSSMultiModalFactor::computeResidualAndCovariance(
    const gtsam::Values& x, std::vector<Matrix>* H) const {
  const nonBiasStates& q = x.at<nonBiasStates>(k1_);
  const PhaseBias& g = x.at<PhaseBias>(k2_);

  const Vector h = obsMap(satXYZ_, nomXYZ_, 1);

  const double res_range = (h.transpose() * q) - measured_[0];
  const double res_phase = (h.transpose() * q) + g[0] - measured_[1];

  double prob_max = -std::numeric_limits<double>::infinity();
  bool found_valid = false;
  Eigen::Vector2d res_best;
  res_best << res_range, res_phase;
  Matrix2 cov_best = Matrix2::Identity();

  // Select the most likely Gaussian component under the current residual.
  for (const auto& mixture : gmm_) {
    Eigen::Vector2d res_local;
    res_local << res_range - mixture.mean(0), res_phase - mixture.mean(1);

    const Matrix2 cov = mixture.cov;
    Eigen::LLT<Matrix2> llt(cov);
    if (llt.info() != Eigen::Success) {
      continue;
    }
    const Eigen::Vector2d solved = llt.solve(res_local);
    const double quad = res_local.dot(solved);
    const double sqrt_det = llt.matrixL().diagonal().array().abs().prod();
    const double det = std::max(
        sqrt_det * sqrt_det, std::numeric_limits<double>::min());
    const double norm = std::pow(std::sqrt(2.0 * kPi), -1) * std::pow(det, -0.5);
    const double prob = norm * std::exp(-0.5 * quad);

    if (prob >= prob_max) {
      found_valid = true;
      prob_max = prob;
      cov_best = cov;
      res_best = res_local;
    }
  }

  if (!found_valid) {
    throw GnssInvalidModelError(
        "No valid positive-definite Gaussian mixture component available");
  }

  if (H) {
    Matrix H_g(2, 5);
    H_g.row(0) = h;
    H_g.row(1) = h;

    Matrix H_b(2, 1);
    H_b(0, 0) = 0.0;
    H_b(1, 0) = 1.0;

    (*H)[0].resize(H_g.rows(), H_g.cols());
    (*H)[1].resize(H_b.rows(), H_b.cols());

    (*H)[0] = H_g;
    (*H)[1] = H_b;
  }

  return {res_best, cov_best};
}

Vector GNSSMultiModalFactor::unwhitenedError(
    const gtsam::Values& x, std::vector<Matrix>*  H) const {
  return computeResidualAndCovariance(x, H).first;
}

void GNSSMultiModalFactor::print(
    const std::string& s, const KeyFormatter& keyFormatter) const {
  std::cout << s << "GNSSMultiModalFactor(" << keyFormatter(k1_) << ","
            << keyFormatter(k2_) << ")\n"
            << "  measured:  " << measured_.transpose();
}

Vector GNSSMultiModalFactor::whitenedError(
    const gtsam::Values& x, std::vector<Matrix>*  H) const {
  const auto residual_cov = computeResidualAndCovariance(x, H);
  return (gtsam::noiseModel::Gaussian::Covariance(residual_cov.second))
      ->whiten(residual_cov.first);
}

} // namespace gtsam
