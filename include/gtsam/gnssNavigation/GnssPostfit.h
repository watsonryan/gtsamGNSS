/**
 *  @file   GNSSPostfit.h
 *  @author Watson
 *  @brief  Header file for GNSS postfit analysis
 **/

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point4.h>
#include <gtsam/gnssNavigation/GnssData.h>
#include <gtsam/gnssNavigation/GnssTools.h>

namespace gtsam {

/// Function to calculate gnss postfit residuals
[[nodiscard]] std::vector<double> getResiduals(const Point3& nomXYZ, const Values& results, const std::vector<rnxData>& data);

/// write residuals to text file
void writeResiduals(const std::vector<double>& postfitResiduals, const std::string& outputFile, const std::vector<std::string>& switchIndex);

// iterate over residual vector to mark outliers.
[[nodiscard]] std::vector<int> markResiduals(const std::vector<double>& postfitResdiuals, double threshold);

}
