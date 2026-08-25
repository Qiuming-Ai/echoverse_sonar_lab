#pragma once
// Excitation + matched filter generation (port of SonarInit.m tx branches)
// and sonar attribute building (port of DataMakerInit.m).
//
// tx_type 'lfm' | 'cdm' | 'fdm' fully implemented (golden-calibrated).
#include <string>

#include "types.h"

namespace sonar::sim {

// Fill s.exc_nt ([Ntx x M], complex analytic excitation per element) and
// s.MF  ([M x 1], decimated matched filter MF_deci).
// Also returns the intermediate (non-decimated) matched filter for reference.
void generate_excitation(SonarConfig& s);

// Build the HDF5 header attributes (port of DataMakerInit sonarInfo struct).
SonarAttributes build_sonar_attributes(const SonarConfig& s, const std::string& timestamp);

}  // namespace sonar::sim

