/**
 *  @file   PhaseSwitchFactor.cpp
 *  @author Watson
 *  @brief  Implementation file for carrier-phase factor
 **/

#include <gtsam/robustModels/PhaseSwitchFactor.h>

namespace gtsam {
//***************************************************************************
Vector PhaseSwitchFactor::evaluateError(
    const nonBiasStates& q, const PhaseBias& g,
    const vertigo::SwitchVariableLinear& s,
    HNonBias H1,
    HPhaseBias H2,
    HSwitch H3) const {

        Vector h = obsMap(satXYZ_, nomXYZ_, 1);
        double est = (h.transpose() * q) + g[0];
        const double residual = est - measured_;
        Vector error = (Vector(1) << residual).finished();
        error *= s.value();

        if (H1) { (*H1) = (Matrix(1,5) << h * s.value() ).finished(); }
        Matrix gnssPartials = Z_1x1;
        if (H2) {
                gnssPartials(0) = 1.0 * s.value(); // phase bias
                (*H2) = gnssPartials;
        }
        if (H3) { (*H3) = (Matrix(1,1) << residual).finished(); }
        return (Matrix(1,1) << error ).finished();
}

}  //namespace
