/**
 *  @file   PseudorangeMaxMix.cpp
 *  @author Watson
 *  @brief  Implementation file for pseudorange max-mix factor
 **/

#include <gtsam/robustModels/PseudorangeMaxMix.h>

#include <algorithm>
#include <limits>

namespace gtsam {

//***************************************************************************
Vector PseudorangeMaxMix::evaluateError(const nonBiasStates& q,
                                        OptionalMatrixType H1 ) const {

        auto error = ( Vector(1) << h_.transpose()*q-measured_ ).finished();

        auto g1 = noiseModel::Gaussian::Covariance(
                (( Vector(1) << hyp_).finished()).asDiagonal());
        auto g2 = noiseModel::Gaussian::Covariance(
                (( Vector(1) << hyp_/w_).finished()).asDiagonal());

        double m1 = this->noiseModel()->distance(error);
        Matrix info1(g1->information());
        const double info_det1 =
            std::max(info1.determinant(), std::numeric_limits<double>::min());
        double nu1 = std::sqrt(info_det1);
        double l1 = nu1 * std::exp(-0.5 * m1);

        double m2 = nullHypothesisModel_->distance(error);
        Matrix info2(g2->information());
        const double info_det2 =
            std::max(info2.determinant(), std::numeric_limits<double>::min());
        double nu2 = std::sqrt(info_det2);
        double l2 = nu2 * std::exp(-0.5 * m2);

        if (H1) { (*H1) = (Matrix(1,5) << h_.transpose() ).finished(); }

        if (l2 > l1) {
                if (H1) *H1 = *H1 * w_;
                error *= std::sqrt(w_);
        }

        return (Vector(1) << error).finished();
}
} // namespace
