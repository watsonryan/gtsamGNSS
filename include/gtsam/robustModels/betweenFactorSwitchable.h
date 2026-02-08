/*
 * betweenFactorSwitchable.h
 *
 *  Created on: 02.08.2012
 *      Author: niko
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/robustModels/switchVariableLinear.h>
#include <gtsam/slam/BetweenFactor.h>

namespace gtsam {

template <class VALUE>
class BetweenFactorSwitchableLinear
    : public NoiseModelFactor3<VALUE, VALUE, vertigo::SwitchVariableLinear> {
 public:
  BetweenFactorSwitchableLinear() = default;

  BetweenFactorSwitchableLinear(Key key1, Key key2, Key key3,
                                const VALUE& measured,
                                const SharedNoiseModel& model)
      : NoiseModelFactor3<VALUE, VALUE, vertigo::SwitchVariableLinear>(model,
                                                                        key1,
                                                                        key2,
                                                                        key3),
        betweenFactor(key1, key2, measured, model) {}

  Vector evaluateError(const VALUE& p1, const VALUE& p2,
                       const vertigo::SwitchVariableLinear& s,
                       OptionalMatrixType H1 = OptionalNone,
                       OptionalMatrixType H2 = OptionalNone,
                       OptionalMatrixType H3 = OptionalNone) const {
    Vector error = betweenFactor.evaluateError(p1, p2, H1, H2);
    error *= s.value();

    if (H1) *H1 = *H1 * s.value();
    if (H2) *H2 = *H2 * s.value();
    if (H3) *H3 = error;

    return error;
  }

 private:
  BetweenFactor<VALUE> betweenFactor;
};

}  // namespace gtsam
