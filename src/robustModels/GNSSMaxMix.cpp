/**
 *  @file   GNSSMaxMix.cpp
 *  @author Watson
 *  @brief  Implementation file for pseudorange max-mix factor
 **/

#include <gtsam/robustModels/GNSSMaxMix.h>

#include <algorithm>
#include <iostream>
#include <limits>

namespace gtsam {

//***************************************************************************
void GNSSMaxMix::print(const std::string& s,
                       const KeyFormatter& keyFormatter) const {
        std::cout << s << "GNSS Factor("
                  << keyFormatter(this->key1()) << ","
                  << keyFormatter(this->key2()) << ")\n"
                  << "  measured:  " << measured_.transpose() << "\n"
                  << " noise model: ";
        this->noiseModel_->print("  noise model: ");
}

Vector GNSSMaxMix::evaluateError(const nonBiasStates& q, const PhaseBias& g, OptionalMatrixType H1, OptionalMatrixType H2) const {

        Vector h = obsMap(satXYZ_, nomXYZ_, 1);

        double res_range = (h.transpose() * q) - measured_[0];
        double res_phase = (h.transpose() * q) + g[0] - measured_[1];
        auto error = (Vector(2) << res_range, res_phase).finished();

        noiseModel::Diagonal::shared_ptr hypothesis = noiseModel::Diagonal::Variances(variances_);
        noiseModel::Diagonal::shared_ptr null = noiseModel::Diagonal::Variances(variances_/w_);


        const double m1 = hypothesis->distance(error);
        const gtsam::Matrix info1(hypothesis->information());
        const double info_det1 = std::max(info1.determinant(), std::numeric_limits<double>::min());
        const double nu1 = std::sqrt(info_det1);
        const double l1 = nu1 * std::exp(-0.5 * m1);

        const double m2 = null->distance(error);
        const gtsam::Matrix info2(null->information());
        const double info_det2 = std::max(info2.determinant(), std::numeric_limits<double>::min());
        const double nu2 = std::sqrt(info_det2);
        const double l2 = nu2 * std::exp(-0.5 * m2);

        // cout << "Error = " <<  error.transpose() << "  --- l1 = " << l1 << " " << "l2 = " << l2 << endl;

        if (H1)
        {
                Matrix H_g(2,5);
                H_g.row(0) = h;
                H_g.row(1) = h;
                (*H1) = H_g;
        }
        if (H2)
        {
                Matrix H_b(2,1);
                H_b(0,0) = 0.0;
                H_b(1,0) = 1.0;
                (*H2) = H_b;
        }

        if (l2 > l1) {
                if (H1) *H1 = *H1 * w_;
                error *= std::sqrt(w_);
        }

        return (Vector(2) << error).finished();
}
} // namespace
