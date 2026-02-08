/**
 *  @file   GNSSSwitch.h
 *  @author Watson
 *  @brief  Header file for GNSS switch factor (i.e., a switchable factor for Psueodrange and Carrier-Phase observations)
 **/

#pragma once
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>
#include <gtsam/gnssNavigation/GnssTypes.h>
#include <gtsam/robustModels/switchPairLinear.h>
#include <gtsam/robustModels/switchVariableLinear.h>

namespace gtsam {


class GTSAM_EXPORT GNSSSwitch : public NoiseModelFactor3<nonBiasStates, PhaseBias, vertigo::SwitchPairLinear> {

private:

Vector2 measured_;
Point3 satXYZ_, nomXYZ_;
typedef NoiseModelFactor3<nonBiasStates, PhaseBias, vertigo::SwitchPairLinear> Base;
using HNonBias = typename Base::template OptionalMatrix<nonBiasStates>;
using HPhaseBias = typename Base::template OptionalMatrix<PhaseBias>;
using HSwitchPair = typename Base::template OptionalMatrix<vertigo::SwitchPairLinear>;

public:

typedef std::shared_ptr<GNSSSwitch> shared_ptr;
typedef GNSSSwitch This;

GNSSSwitch() : measured_() {
}

virtual ~GNSSSwitch() {
}

GNSSSwitch(Key a, Key b, Key c, const Vector2 measurement,
           const Point3 satXYZ, const Point3 nomXYZ, const SharedNoiseModel &model) :
        Base(model, a,b,c), measured_(measurement), satXYZ_(satXYZ), nomXYZ_(nomXYZ) {
}


/** print */
virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return gtsam::NonlinearFactor::shared_ptr(new GNSSSwitch(*this));
}

Vector evaluateError(const nonBiasStates& q, const PhaseBias& g,
                     const vertigo::SwitchPairLinear& s,
                     HNonBias H1 = boost::none,
                     HPhaseBias H2 = boost::none,
                     HSwitchPair H3 = boost::none) const;

private:

}; // GNSSSwitch Factor
} // namespace
