/**
 *  @file   FolderUtils.h
 *  @author Watson
 *  @brief  Header file to allow for simple manipulation of directories.
 **/

#pragma once

#include <string>

namespace gtsam {

std::string getTimestamp();

void makeDir(const std::string& dir);

} // namespace gtsam
