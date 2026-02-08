/**
 *  @file   GNSSMultiModalFactor.h
 *  @author Watson
 *  @brief  GNSS factor with a Gaussian-mixture residual model
 **/

#pragma once

#include <utility>
#include <vector>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/gnssNavigation/GnssTypes.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

class GTSAM_EXPORT GNSSMultiModalFactor : public NonlinearFactor {
private:
  using Base = gtsam::NonlinearFactor;
  using This = GNSSMultiModalFactor;

  Key k1_{0};
  Key k2_{0};
  Point3 satXYZ_;
  Point3 nomXYZ_;
  nonBiasStates h_;
  Vector2 measured_;
  std::vector<GaussianMixtureComponent> gmm_;

public:
  using shared_ptr = gtsam::NonlinearFactor::shared_ptr;

  GNSSMultiModalFactor() : measured_() { h_ = Matrix(2, 5); }

  GNSSMultiModalFactor(Key deltaStates, Key bias, const Vector2 measurement,
                       const Point3 satXYZ, const Point3 nomXYZ,
                       const std::vector<GaussianMixtureComponent>& gmm)
      : Base(KeyVector{deltaStates, bias}),
        k1_(deltaStates),
        k2_(bias),
        satXYZ_(satXYZ),
        nomXYZ_(nomXYZ),
        measured_(measurement),
        gmm_(gmm) {
    validateGmmOrThrow(gmm_);
  }

  ~GNSSMultiModalFactor() override = default;

  void print(const std::string& s,
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  bool equals(const NonlinearFactor& f, double tol = 1e-9) const override {
    const This* t = dynamic_cast<const This*>(&f);
    if (t && Base::equals(f, tol)) {
      return k1_ == t->k1_ && k2_ == t->k2_;
    }
    return false;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new GNSSMultiModalFactor(*this));
  }

  double error(const gtsam::Values& x) const override {
    return whitenedError(x).squaredNorm();
  }

  Vector unwhitenedError(const gtsam::Values& x,
                         std::vector<Matrix>* H = nullptr) const;

  Vector whitenedError(const gtsam::Values& x,
                       std::vector<Matrix>* H = nullptr) const;

  Vector residual(const gtsam::Values& x) const {
    return unwhitenedError(x);
  }

  size_t dim() const override { return 5; }

  std::size_t size() const { return 2; }

  bool active(const gtsam::Values& x) const override {
    (void)x;
    return true;
  }

  gtsam::GaussianFactor::shared_ptr linearize(
      const gtsam::Values& x) const override {
    if (!active(x))
      return gtsam::GaussianFactor::shared_ptr();

    std::vector<Matrix> A(this->size());
    const auto residual_cov = computeResidualAndCovariance(x, &A);
    const Vector b = residual_cov.first;
    const Matrix2 cov = residual_cov.second;

    std::vector<std::pair<Key, Matrix>> terms(size());
    for (size_t j = 0; j < size(); ++j) {
      terms[j].first = keys()[j];
      terms[j].second.swap(A[j]);
    }

    return GaussianFactor::shared_ptr(new JacobianFactor(
        terms, -b,
        noiseModel::Diagonal::Variances(
            (gtsam::Vector(2) << cov(0, 0), cov(1, 1)).finished())));
  }

private:
  static void validateGmmOrThrow(const std::vector<GaussianMixtureComponent>& gmm);
  std::pair<Vector, Matrix2> computeResidualAndCovariance(
      const gtsam::Values& x, std::vector<Matrix>* H) const;
};

} // namespace gtsam
