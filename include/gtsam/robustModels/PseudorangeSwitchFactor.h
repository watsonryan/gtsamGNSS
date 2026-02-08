/**
 *  @file   PseudorangeSwitchFactor.h
 *  @author Watson
 *  @brief  Header file for Pseudorange Switchable factor
 **/

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>
#include <gtsam/robustModels/switchVariableLinear.h>



namespace gtsam {

class GTSAM_EXPORT PseudorangeSwitchFactor : public NoiseModelFactor2<nonBiasStates,vertigo::SwitchVariableLinear> {

private:
typedef NoiseModelFactor2<nonBiasStates,vertigo::SwitchVariableLinear> Base;
Point3 nomXYZ_;
Point3 satXYZ_;
double measured_;
nonBiasStates h_;

public:

typedef std::shared_ptr<PseudorangeSwitchFactor> shared_ptr;
typedef PseudorangeSwitchFactor This;

PseudorangeSwitchFactor() : measured_(0) {
        h_=(Matrix(1,5)<<1,1,1,1,1).finished();
}

virtual ~PseudorangeSwitchFactor() {
}

PseudorangeSwitchFactor(Key j, Key k, const double deltaObs, const Point3 satXYZ, const Point3 nomXYZ, const SharedNoiseModel& model) :
        Base(model, j,k), measured_(deltaObs), satXYZ_(satXYZ) {
        nomXYZ_=nomXYZ;
}
virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return gtsam::NonlinearFactor::shared_ptr(new PseudorangeSwitchFactor(*this));
}

/// vector of errors
Vector evaluateError(const nonBiasStates& q,
                     const vertigo::SwitchVariableLinear& s,
                     OptionalMatrixType H1 = OptionalNone,
                     OptionalMatrixType H2 = OptionalNone) const;

private:

}; // PseudorangeSwitchFactor Factor
} // namespace
