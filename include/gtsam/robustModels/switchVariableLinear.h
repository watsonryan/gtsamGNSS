#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Lie.h>

#include <cstddef>
#include <cmath>
#include <string>

namespace vertigo {

// SwitchVariableLinear wraps a scalar switch state constrained to [0, 1].
struct SwitchVariableLinear {
  SwitchVariableLinear() : d_(0.0) {}
  explicit SwitchVariableLinear(double d) : d_(d) {}

  double value() const { return d_; }

  void print(const std::string& name = "") const;

  inline bool equals(const SwitchVariableLinear& expected, double tol = 1e-5) const {
    return std::abs(expected.d_ - d_) <= tol;
  }

  inline size_t dim() const { return 1; }
  inline static size_t Dim() { return 1; }

  inline SwitchVariableLinear retract(const gtsam::Vector& v) const {
    double x = value() + v(0);
    if (x > 1.0) x = 1.0;
    else if (x < 0.0) x = 0.0;
    return SwitchVariableLinear(x);
  }

  inline gtsam::Vector localCoordinates(const SwitchVariableLinear& t2) const {
    return gtsam::Vector1(t2.value() - value());
  }

  inline static SwitchVariableLinear identity() { return SwitchVariableLinear(); }

  inline SwitchVariableLinear compose(const SwitchVariableLinear& p) const {
    return SwitchVariableLinear(d_ + p.d_);
  }

  inline SwitchVariableLinear between(const SwitchVariableLinear& l2,
                                      gtsam::OptionalJacobian<1, 1> H1 = boost::none,
                                      gtsam::OptionalJacobian<1, 1> H2 = boost::none) const {
    if (H1) *H1 = -gtsam::Matrix::Identity(1, 1);
    if (H2) *H2 = gtsam::Matrix::Identity(1, 1);
    return SwitchVariableLinear(l2.value() - value());
  }

  inline SwitchVariableLinear inverse() const {
    return SwitchVariableLinear(-1.0 * value());
  }

  static inline SwitchVariableLinear Expmap(const gtsam::Vector& v) {
    return SwitchVariableLinear(v(0));
  }

  static inline gtsam::Vector Logmap(const SwitchVariableLinear& p) {
    return gtsam::Vector1(p.value());
  }

 private:
  double d_;
};

}  // namespace vertigo

namespace gtsam {

template <typename T>
struct traits;

template <>
struct traits<vertigo::SwitchVariableLinear> {
  static void Print(const vertigo::SwitchVariableLinear& key,
                    const std::string& str = "") {
    key.print(str);
  }

  static bool Equals(const vertigo::SwitchVariableLinear& key1,
                     const vertigo::SwitchVariableLinear& key2,
                     double tol = 1e-8) {
    return key1.equals(key2, tol);
  }

  static int GetDimension(const vertigo::SwitchVariableLinear& key) {
    return static_cast<int>(key.Dim());
  }

  using ChartJacobian = OptionalJacobian<1, 1>;
  using TangentVector = gtsam::Vector;

  static TangentVector Local(const vertigo::SwitchVariableLinear& origin,
                             const vertigo::SwitchVariableLinear& other,
                             ChartJacobian Horigin = boost::none,
                             ChartJacobian Hother = boost::none) {
    (void)Horigin;
    (void)Hother;
    return origin.localCoordinates(other);
  }

  static vertigo::SwitchVariableLinear Retract(
      const vertigo::SwitchVariableLinear& g,
      const TangentVector& v,
      ChartJacobian H1 = boost::none,
      ChartJacobian H2 = boost::none) {
    (void)H1;
    (void)H2;
    return g.retract(v);
  }
};

}  // namespace gtsam
