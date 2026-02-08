/**
 *  @file   PseudorangeMaxMix.h
 *  @author Watson
 *  @brief  Header file for Pseudorange Max-Mix factor
 **/

#pragma once

#include <Eigen/Eigen>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>


namespace gtsam {

class GTSAM_EXPORT PseudorangeMaxMix : public NoiseModelFactor1<nonBiasStates> {

private:
typedef NoiseModelFactor1<nonBiasStates> Base;
using HNonBias = typename Base::template OptionalMatrix<nonBiasStates>;
double measured_, w_, hyp_;
nonBiasStates h_;
SharedNoiseModel nullHypothesisModel_;


public:

typedef std::shared_ptr<PseudorangeMaxMix> shared_ptr;
typedef PseudorangeMaxMix This;

PseudorangeMaxMix() : measured_(0) {
        h_=(Matrix(1,5)<<1,1,1,1,1).finished();
}

virtual ~PseudorangeMaxMix() {
}

PseudorangeMaxMix(Key key, const double deltaObs, const Matrix obsMap,
                  const SharedNoiseModel& model1, const SharedNoiseModel& model2,
                  const double& hypNoise, double weight) : NoiseModelFactor1<nonBiasStates>(model1, key),
        measured_(deltaObs), h_(obsMap), w_(weight), hyp_(hypNoise),
        nullHypothesisModel_(model2) {
};

virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return gtsam::NonlinearFactor::shared_ptr(new PseudorangeMaxMix(*this));
}

/// vector of errors
Vector evaluateError(const nonBiasStates& q,
                     HNonBias H1 = boost::none) const;

private:

}; // PseudorangeMaxMix Factor
} // namespace
