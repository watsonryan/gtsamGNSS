/**
 * @file   FolderUtils.cpp
 * @brief  Tools to allow for simple manipulation of directories.
 *  @author Watson
 */

#include <gtsam/gnssNavigation/FolderUtils.h>

#include <algorithm>
#include <ctime>
#include <filesystem>

namespace gtsam {

std::string getTimestamp() {
  const auto now = std::time(nullptr);
  char buf[sizeof("YYYY-MM-DD  HH:MM:SS")];
  std::string timeDateStr =
      std::string(buf, buf + std::strftime(buf, sizeof(buf), "%F  %T", std::gmtime(&now)));
  if (const auto pos = timeDateStr.find("  "); pos != std::string::npos) {
    timeDateStr.replace(pos, 2, "_");
  }
  std::replace(timeDateStr.begin(), timeDateStr.end(), ':', '-');
  return timeDateStr;
}

void makeDir(const std::string& dir) {
  std::filesystem::create_directories(dir);
}

} // namespace gtsam
