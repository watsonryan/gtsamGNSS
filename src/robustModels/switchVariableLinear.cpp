/**
 *  @file   switchVariableLinear.cpp
 *  @author Watson
 *  @brief  Implementation for SwitchVariableLinear printing
 **/

#include <gtsam/robustModels/switchVariableLinear.h>

#include <iostream>

namespace vertigo {

void SwitchVariableLinear::print(const std::string& name) const {
  std::cout << name << ": " << d_ << '\n';
}

}  // namespace vertigo
