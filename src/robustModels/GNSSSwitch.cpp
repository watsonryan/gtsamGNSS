/**
 *  @file   GNSSSwitch.cpp
 *  @author Watson
 *  @brief  Implementation file for GNSS switch factor
 **/

#include <gtsam/robustModels/GNSSSwitch.h>

#include <iostream>

//***************************************************************************
namespace gtsam {
void GNSSSwitch::print(const std::string& s,
                       const KeyFormatter& keyFormatter) const {
        std::cout << s << "GNSS Factor("
                  << keyFormatter(this->key1()) << ","
                  << keyFormatter(this->key2()) << ")\n"
                  << keyFormatter(this->key3()) << ")\n"
                  << "  measured:  " << measured_.transpose() << "\n"
                  << " noise model: ";
        this->noiseModel_->print("  noise model: ");
}

Vector GNSSSwitch::evaluateError(const nonBiasStates& q, const PhaseBias& g,
                                 const vertigo::SwitchPairLinear& s,
                                 HNonBias H1,
                                 HPhaseBias H2,
                                 HSwitchPair H3) const {

        Vector5 h = obsMap(satXYZ_, nomXYZ_, 1);

        const double residual_range = (h.transpose() * q) - measured_[0];
        const double residual_phase = (h.transpose() * q) + g[0] - measured_[1];
        double res_range = residual_range * s.value()[0];
        double res_phase = residual_phase * s.value()[1];

        Vector error = (Vector(2) << res_range, res_phase).finished();

        if (H1) { (*H1) = (Matrix(2,5) << h*s.value()[0], h*s.value()[1]).finished(); }
        if (H2) { (*H2) = (Matrix(2,1) << 0.0, s.value()[1]).finished(); }
        if (H3) { (*H3) = (Matrix(2,2) << residual_range, 0.0, 0.0, residual_phase).finished(); }
        return error;
}

}  //namespace
