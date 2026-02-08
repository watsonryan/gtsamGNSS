/**
 *  @file   switchPairLinear.cpp
 *  @author Watson
 *  @brief  Implementation for SwitchPairLinear printing
 **/

#include <gtsam/robustModels/switchPairLinear.h>

#include <iostream>

namespace vertigo {

void SwitchPairLinear::print(const std::string& name) const {
  std::cout << name << ": " << v_ << '\n';
}

}  // namespace vertigo
