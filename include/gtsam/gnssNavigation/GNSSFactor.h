/**
 *  @file   GNSSFactor.h
 *  @author Watson
 *  @brief  Header file for GNSS Factor (i.e., a factor for Psueodrange and Carrier-Phase observations)
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

namespace gtsam {


class GTSAM_EXPORT GNSSFactor : public NoiseModelFactor2<nonBiasStates, PhaseBias> {

private:
typedef NoiseModelFactor2<nonBiasStates, PhaseBias> Base;
using HNonBias = typename Base::template OptionalMatrix<nonBiasStates>;
using HPhaseBias = typename Base::template OptionalMatrix<PhaseBias>;
Point3 satXYZ_;
Point3 nomXYZ_;
Vector2 measured_;
nonBiasStates h_;

public:

typedef std::shared_ptr<GNSSFactor> shared_ptr;
typedef GNSSFactor This;

GNSSFactor() : measured_() {
        h_=Matrix(2,5);
}

virtual ~GNSSFactor() {
}

GNSSFactor(Key deltaStates, Key bias, const Vector2 measurement,
           const Point3 satXYZ, const Point3 nomXYZ, const SharedNoiseModel &model) :
        Base(model, deltaStates, bias), measured_(measurement)
{
        satXYZ_=satXYZ;
        nomXYZ_=nomXYZ;
}


/** print */
virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return gtsam::NonlinearFactor::shared_ptr(new GNSSFactor(*this));
}

Vector evaluateError(const nonBiasStates& q, const PhaseBias& g,
                     HNonBias H1 = boost::none,
                     HPhaseBias H2 = boost::none ) const;

private:

}; // GNSSFactor Factor
} // namespace
