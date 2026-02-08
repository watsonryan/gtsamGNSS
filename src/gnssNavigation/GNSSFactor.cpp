/**
 *  @file   GNSSFactor.cpp
 *  @author Watson
 *  @brief  Implementation file for GNSS factor
 **/

#include <gtsam/gnssNavigation/GNSSFactor.h>

#include <iostream>

namespace gtsam {
//***************************************************************************
void GNSSFactor::print(const std::string& s,
                       const KeyFormatter& keyFormatter) const {
        std::cout << s << "GNSS Factor("
                  << keyFormatter(this->key1()) << ","
                  << keyFormatter(this->key2()) << ")\n"
                  << "  measured:  " << measured_.transpose() << "\n"
                  << " noise model: ";
        this->noiseModel_->print("  noise model: ");
}

Vector GNSSFactor::evaluateError(const nonBiasStates& q, const PhaseBias& g,
                                 HNonBias H1,
                                 HPhaseBias H2) const {

        Vector h = obsMap(satXYZ_, nomXYZ_, 1);

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
        double res_range = (h.transpose() * q) - measured_[0];
        double res_phase = (h.transpose() * q) + g[0] - measured_[1];

        return (Vector(2) << res_range, res_phase).finished();
}

}  //namespace
