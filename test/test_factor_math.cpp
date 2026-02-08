#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/gnssNavigation/GNSSFactor.h>
#include <gtsam/gnssNavigation/GNSSMultiModalFactor.h>
#include <gtsam/gnssNavigation/GnssTypes.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>

namespace {

using gtsam::Point3;
using gtsam::Vector2;

void test_gnss_factor_residual_matches_model() {
  const gtsam::Key k_state = gtsam::Symbol('x', 0);
  const gtsam::Key k_bias = gtsam::Symbol('b', 0);

  const Point3 sat(20200000.0, 1000.0, 1000.0);
  const Point3 nom(1113194.9, -4841695.5, 3985350.2);
  const auto model = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(2) << 1.0, 1.0).finished());

  gtsam::nonBiasStates q(1.0, 2.0, 3.0, 0.4, 0.1);
  gtsam::PhaseBias b;
  b << 0.2;

  const auto h = gtsam::obsMap(sat, nom, 1);
  const double est_range = h.transpose() * q;
  const double est_phase = est_range + b[0];

  const Vector2 meas(est_range - 5000.0, est_phase + 3000.0);
  gtsam::GNSSFactor f(k_state, k_bias, meas, sat, nom, model);

  const auto err = f.evaluateError(q, b);
  const double expected_range = est_range - meas(0);
  const double expected_phase = est_phase - meas(1);
  assert(std::isfinite(err(0)));
  assert(std::isfinite(err(1)));
  assert(std::abs(err(0) - expected_range) < 1e-6);
  assert(std::abs(err(1) - expected_phase) < 1e-6);
}

void test_multimodal_factor_selects_closest_component() {
  const gtsam::Key k_state = gtsam::Symbol('x', 1);
  const gtsam::Key k_bias = gtsam::Symbol('b', 1);

  const Point3 sat(21000000.0, -3000.0, 800.0);
  const Point3 nom(1113194.9, -4841695.5, 3985350.2);

  gtsam::nonBiasStates q(2.0, -1.0, 0.5, 0.1, 0.0);
  gtsam::PhaseBias b;
  b << 0.3;

  const auto h = gtsam::obsMap(sat, nom, 1);
  const double est_range = h.transpose() * q;
  const double est_phase = est_range + b[0];

  Vector2 meas(est_range - 2000.0, est_phase + 1000.0);

  gtsam::GaussianMixtureComponent c1;
  c1.weight = 0.6;
  c1.mean = (Eigen::RowVector2d() << 2000.0, -1000.0).finished();
  c1.cov = 0.5 * Eigen::Matrix2d::Identity();

  std::vector<gtsam::GaussianMixtureComponent> gmm{c1};
  gtsam::GNSSMultiModalFactor f(k_state, k_bias, meas, sat, nom, gmm);

  gtsam::Values values;
  values.insert(k_state, q);
  values.insert(k_bias, b);

  const auto unwhitened = f.unwhitenedError(values);
  assert(std::isfinite(unwhitened(0)));
  assert(std::isfinite(unwhitened(1)));
  assert(std::abs(unwhitened(0)) < 1e-6);
  assert(std::abs(unwhitened(1)) < 1e-6);
}

} // namespace

int main() {
  test_gnss_factor_residual_matches_model();
  test_multimodal_factor_selects_closest_component();
  std::cout << "gtsamGNSS factor math tests passed\n";
  return 0;
}
