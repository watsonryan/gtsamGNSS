/**
 * @file   GnssData.h
 * @brief  Tools required to read/write GNSS data
 *  @author Watson
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/config.h>
#include <gtsam/dllexport.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/gnssNavigation/GnssTypes.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>
#include <gtsam/robustModels/GNSSSwitch.h>
#include <gtsam/slam/dataset.h>


#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>


namespace gtsam {

struct RnxData {
  double sow{0.0};
  int epoch{0};
  int svn{0};
  Point3 sat_xyz;
  double computed_range{0.0};
  double range_lc{0.0};
  double phase_lc{0.0};
  int break_flag{0};
};
using rnxData = RnxData;
[[nodiscard]] std::vector<rnxData> readGNSS(const std::string& fileLoc);
[[nodiscard]] std::vector<rnxData> readGNSS_SingleFreq(const std::string& fileLoc);

struct RnxDataWEl {
  double sow{0.0};
  int epoch{0};
  int svn{0};
  Point3 sat_xyz;
  double computed_range{0.0};
  double range_lc{0.0};
  double phase_lc{0.0};
  double break_flag{0.0};
  double elevation{0.0};
};
using rnxDataWEl = RnxDataWEl;
[[nodiscard]] std::vector<rnxDataWEl> readGNSS_SingleFreqTmp(const std::string& fileLoc);

/// Read GNSS data and add faults to observations.
struct FaultyRnxData {
  double sow{0.0};
  int epoch{0};
  int svn{0};
  Point3 sat_xyz;
  double computed_range{0.0};
  double range_lc{0.0};
  double phase_lc{0.0};
  int break_flag{0};
  int fault_flag{0};
};
using faultyRnxData = FaultyRnxData;
[[nodiscard]] std::vector<faultyRnxData> readGNSSFaulty(
    const std::string& fileLoc, const double& mean, const double& stdDev,
    const double& percentFaulty,
    const std::optional<std::uint32_t>& seed = std::nullopt);
[[nodiscard]] std::vector<faultyRnxData> readGNSSOracle(
    const std::string& fileLoc, const double& mean, const double& stdDev,
    const double& percentFaulty,
    const std::optional<std::uint32_t>& seed = std::nullopt);

void writeStates(const Values& results, const std::vector<std::string>& timeIndex,
                 const std::string& outputFile);
void writeNavFrame(const Values& results, const Point3& nom,
                   const std::vector<std::string>& timeIndex,
                   const std::string& outputFile);
void writeEarthFrame(const Values& results, const Point3& nom,
                     const std::vector<std::string>& timeIndex,
                     const std::string& outputFile);
void writeSwitches(const Values& results, const std::string& outputFile,
                   const std::vector<std::string>& switchIndex);
void writeSwitchPair(const Values& results, const std::string& outputFile,
                     const std::vector<std::string>& switchIndex);
void writeAmbiguity(const Values& results, const std::string& outputFile,
                    const std::vector<std::string>& satIndex);

} // namespace gtsam
