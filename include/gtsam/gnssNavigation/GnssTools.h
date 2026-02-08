/**
 * @file   GnssTools.h
 * @brief  Tools required to process GNSS data -- (i.e. ECEF to ENU transformation)
 *  @author Watson
 */

#pragma once

#include <gtsam/config.h>
#include <gtsam/dllexport.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/gnssNavigation/PhysicalConstants.h>

#include <cmath>


namespace gtsam {

//// rotate from ECI to ECEF
[[nodiscard]] Point3 inertialToECEF( const Point3& inertialPosition, const double t, const double t0);

//// Generate rotation matrix from Earth-to-Navigation frame
[[nodiscard]] Matrix earthToNavTrans( const Point3& ECEFxyz);

//// Compute mapping from meas. to states
[[nodiscard]] Vector obsMap(const Point3& p1, const Point3& q, const int& Trop = 0);

//// Extract PRN Vector from GNSS data structure
[[nodiscard]] Eigen::VectorXi getPRN(const Matrix& p);

//// See if current PRN value was present at previous epoch
[[nodiscard]] bool checkPRN(const Eigen::VectorXi& p, const int& n);

/// computer delta pseudorange observables
[[nodiscard]] double deltaObs(const Point3& p1, const Point3& p2, const double& pseudorange);

/// compute the delta troposphere correction
[[nodiscard]] double deltaTrop(const Point3& p1, const Point3& p2);

//// Convert from WGS-84 ECEF coordinated to local-level-tangent (ENU) coordinates
////
//// REF :: Groves, Paul. Principles of GNSS, Inertial, and Multisensor Integrated
////        Navigation Systems. Artech House, 2008
[[nodiscard]] Point3 xyz2enu(const Point3& p1, const Point3& p2);

//// Convert WGS-84 ECEF coordinates to LLH
////
//// REF :: Groves, Paul. Principles of GNSS, Inertial, and Multisensor Integrated
////        Navigation Systems. Artech House, 2008
[[nodiscard]] Point3 xyz2llh(const Point3& p1);

//// Convert ENU coordinates to ECEF
////
//// REF :: Groves, Paul. Principles of GNSS, Inertial, and Multisensor Integrated
////        Navigation Systems. Artech House, 2008
[[nodiscard]] Point3 enu2xyz(const Point3& p1, const Point3& p2);

/// Convert NED Local Frame to ENU Local Frame
[[nodiscard]] Point3 ned2enu(const Point3& p1);

//// Computer Elevation of Satellite from Receiver
[[nodiscard]] double calcEl(const Point3& p1, const Point3& p2);

//// Compute elevation angle given a NED position vector.
[[nodiscard]] double calcElNed(const Point3& p1);

//// Computer elevation angle dependant weighting.
[[nodiscard]] double elDepWeight(const Point3& p1, const Point3& p2, double measWeight);

//// Elevation angle only troposphere mapping
////
//// REF :: Black, H. and Eisner, A., 1984. Correcting satellite Doppler data for
////        tropospheric effects. Journal of Geophysical Research.
[[nodiscard]] double tropMap(const double& El);

//// Troposphere Model --- dry component only. Uses the Sass. model.
////
//// REF :: Saastamoinen, J. 1972, Atmospheric correction for the troposphere and
////        stratosphere in radio ranging of satellites, in The Use of Artificial
////        Satellites for Geodesy
[[nodiscard]] double tropDry(const Point3& p1);

//// time difference carrier-phase observations
[[nodiscard]] double dopplerObs(const Point3& p1, double tdcp1, const Point3& p2, double tdcp2);

}
