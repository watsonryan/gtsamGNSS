/**
 *  @file   GnssErrors.h
 *  @author Watson
 *  @brief  Typed exception taxonomy for GNSS I/O and data validation
 **/

#pragma once

#include <sstream>
#include <stdexcept>
#include <string>

namespace gtsam {

class GnssError : public std::runtime_error {
 public:
  explicit GnssError(const std::string& message) : std::runtime_error(message) {}
};

class GnssDataFileOpenError : public GnssError {
 public:
  explicit GnssDataFileOpenError(const std::string& path)
      : GnssError("Failed to open GNSS data file: " + path) {}
};

class GnssOutputFileOpenError : public GnssError {
 public:
  explicit GnssOutputFileOpenError(const std::string& path)
      : GnssError("Failed to open GNSS output file: " + path) {}
};

class GnssParseError : public GnssError {
 public:
  GnssParseError(const std::string& path, const std::string& detail)
      : GnssError("Failed to parse GNSS data file '" + path + "': " + detail) {}
};

class GnssInvalidModelError : public GnssError {
 public:
  explicit GnssInvalidModelError(const std::string& detail)
      : GnssError("Invalid GNSS model: " + detail) {}
};

class GnssIndexMismatchError : public GnssError {
 public:
  GnssIndexMismatchError(const std::string& name, std::size_t index, std::size_t size)
      : GnssError(BuildMessage(name, index, size)) {}

 private:
  static std::string BuildMessage(const std::string& name, std::size_t index,
                                  std::size_t size) {
    std::ostringstream oss;
    oss << "Insufficient " << name << " entries for GNSS output serialization"
        << " (index=" << index << ", size=" << size << ")";
    return oss.str();
  }
};

}  // namespace gtsam
