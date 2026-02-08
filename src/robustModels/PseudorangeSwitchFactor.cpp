/**
 *  @file   PseudorangeSwitchFactor.cpp
 *  @author Watson
 *  @brief  Implementation file for pseudorange switchable factor
 **/

#include <gtsam/robustModels/PseudorangeSwitchFactor.h>

namespace gtsam {

//***************************************************************************
Vector PseudorangeSwitchFactor::evaluateError(const nonBiasStates& q,
                                              const vertigo::SwitchVariableLinear& s,
                                              OptionalMatrixType H1,
                                              OptionalMatrixType H2) const {

        Vector h_ = obsMap(satXYZ_, nomXYZ_, 1);
        const double residual = (h_.transpose() * q) - measured_;
        double error = residual * s.value();
        if (H1) { (*H1) = (Matrix(1,5) << h_.transpose() * s.value() ).finished(); }
        if (H2) { (*H2) = (Vector(1) << residual).finished(); }
        return (Vector(1) << error).finished();
}
} // namespace
