#pragma once
// Parse Sonar/CDM/FDM JSON config and derive geometry (port of SonarInit.m).
#include <string>

#include "types.h"

namespace sonar::io {

// Throws std::runtime_error on failure.
SonarConfig load_sonar_config(const std::string& json_path);

}  // namespace sonar::io

