/**
 *  @file   PhaseSwitchFactor.h
 *  @author Watson
 *  @brief  Header file for Pseudorange Switchable factor
 **/

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>
#include <gtsam/gnssNavigation/GnssTypes.h>
#include <gtsam/robustModels/switchVariableLinear.h>



namespace gtsam {

class GTSAM_EXPORT PhaseSwitchFactor : public NoiseModelFactor3<nonBiasStates, PhaseBias, vertigo::SwitchVariableLinear> {

private:
typedef NoiseModelFactor3<nonBiasStates, PhaseBias, vertigo::SwitchVariableLinear> Base;
using HNonBias = typename Base::template OptionalMatrix<nonBiasStates>;
using HPhaseBias = typename Base::template OptionalMatrix<PhaseBias>;
using HSwitch = typename Base::template OptionalMatrix<vertigo::SwitchVariableLinear>;
Point3 nomXYZ_;
Point3 satXYZ_;
double measured_;
nonBiasStates h_;

public:

typedef std::shared_ptr<PhaseSwitchFactor> shared_ptr;
typedef PhaseSwitchFactor This;

PhaseSwitchFactor() : measured_(0) {
        h_=(Matrix(1,5)<<1,1,1,1,1).finished();
}

virtual ~PhaseSwitchFactor() {
}

PhaseSwitchFactor(Key a, Key b, Key c, const double deltaObs, const Point3 satXYZ, const Point3 nomXYZ, const SharedNoiseModel& model) :
        Base(model,a,b,c), measured_(deltaObs), satXYZ_(satXYZ) {
        nomXYZ_=nomXYZ;
}
virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return gtsam::NonlinearFactor::shared_ptr(new PhaseSwitchFactor(*this));
}

/// vector of errors
Vector evaluateError(const nonBiasStates& q,
                     const PhaseBias& b,
                     const vertigo::SwitchVariableLinear& s,
                     HNonBias H1 = boost::none,
                     HPhaseBias H2 = boost::none,
                     HSwitch H3 = boost::none) const;

private:

}; // PhaseSwitchFactor Factor
} // namespace
