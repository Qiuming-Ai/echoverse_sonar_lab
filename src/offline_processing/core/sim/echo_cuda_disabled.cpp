#include "sim/echo_simulator.h"

namespace sonar::sim {

void set_cuda_echo_backend(bool, const std::string&) {}

bool cuda_echo_enabled() { return false; }

void enable_cuda_echo_from_env(int, char**) {}

void enable_cuda_echo(int, char**) {}

bool echo_cuda_sim(const MatD&, const MatD&, const MatD&, const MatD&, double,
                   double, const MatC&, const EchoSimOptions&, MatFC&, double&) {
    return false;
}

}  // namespace sonar::sim
