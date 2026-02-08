/**
 *  @file   SwitchPairLinear.h
 *  @author Watson
 *  @brief  Header file for Pair of switch factors (simple modification of switchVariableLinear to work with GNSSFactor)
 **/

#pragma once

#include <gtsam/base/DerivedValue.h>
#include <gtsam/base/Lie.h>

#include <cstddef>
#include <cmath>
#include <string>

namespace vertigo {

struct SwitchPairLinear : public gtsam::DerivedValue<SwitchPairLinear> {
  SwitchPairLinear() : v_(0.0, 0.0) {}
  explicit SwitchPairLinear(gtsam::Vector2 v) : v_(v) {}
  SwitchPairLinear(double x, double y) : v_(x, y) {}

  gtsam::Vector2 value() const { return v_; }
  double a() const { return v_(0); }
  double b() const { return v_(1); }

  void print(const std::string& name = "") const;

  inline bool equals(const SwitchPairLinear& expected, double tol = 1e-5) const {
    return std::abs(expected.a() - v_(0)) <= tol &&
           std::abs(expected.b() - v_(1)) <= tol;
  }

  inline size_t dim() const { return 2; }
  inline static size_t Dim() { return 2; }

  inline SwitchPairLinear retract(const gtsam::Vector& v) const {
    double x = v_(0) + v(0);
    double y = v_(1) + v(1);

    if (x > 1.0) x = 1.0;
    else if (x < 0.0) x = 0.0;

    if (y > 1.0) y = 1.0;
    else if (y < 0.0) y = 0.0;

    return SwitchPairLinear(x, y);
  }

  inline gtsam::Vector localCoordinates(const SwitchPairLinear& t2) const {
    return gtsam::Vector2(t2.value() - value());
  }

  inline static SwitchPairLinear identity() { return SwitchPairLinear(); }

  inline SwitchPairLinear compose(const SwitchPairLinear& p) const {
    return SwitchPairLinear(v_ + p.v_);
  }

  inline SwitchPairLinear between(const SwitchPairLinear& l2,
                                  OptionalMatrixType H1 = OptionalNone,
                                  OptionalMatrixType H2 = OptionalNone) const {
    if (H1) *H1 = -gtsam::eye(2);
    if (H2) *H2 = gtsam::eye(2);
    return SwitchPairLinear(l2.value() - value());
  }

  inline SwitchPairLinear inverse() const {
    return SwitchPairLinear(-1.0 * value());
  }

  static inline SwitchPairLinear Expmap(const gtsam::Vector& v) {
    return SwitchPairLinear(v(0), v(1));
  }

  static inline gtsam::Vector Logmap(const SwitchPairLinear& p) {
    return gtsam::Vector2(p.value());
  }

 private:
  gtsam::Vector2 v_;
};

}  // namespace vertigo

namespace gtsam {

template <typename T>
struct traits;

template <>
struct traits<vertigo::SwitchPairLinear> {
  static void Print(const vertigo::SwitchPairLinear& key,
                    const std::string& str = "") {
    key.print(str);
  }

  static bool Equals(const vertigo::SwitchPairLinear& key1,
                     const vertigo::SwitchPairLinear& key2,
                     double tol = 1e-8) {
    return key1.equals(key2, tol);
  }

  static int GetDimension(const vertigo::SwitchPairLinear& key) {
    return static_cast<int>(key.Dim());
  }

  using ChartJacobian = OptionalJacobian<2, 2>;
  using TangentVector = gtsam::Vector;

  static TangentVector Local(const vertigo::SwitchPairLinear& origin,
                             const vertigo::SwitchPairLinear& other,
                             ChartJacobian Horigin = OptionalNone,
                             ChartJacobian Hother = OptionalNone) {
    (void)Horigin;
    (void)Hother;
    return origin.localCoordinates(other);
  }

  static vertigo::SwitchPairLinear Retract(
      const vertigo::SwitchPairLinear& g,
      const TangentVector& v,
      ChartJacobian H1 = OptionalNone,
      ChartJacobian H2 = OptionalNone) {
    (void)H1;
    (void)H2;
    return g.retract(v);
  }
};

}  // namespace gtsam
