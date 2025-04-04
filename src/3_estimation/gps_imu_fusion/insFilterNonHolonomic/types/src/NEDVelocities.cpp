#include <InsFilterNonHolonomicTypes.hpp>

using namespace InsFilterNonHolonomicTypes;

NEDVelocities::NEDVelocities(double vel_N, double vel_E, double vel_D)
{
    this->vel_N = vel_N;
    this->vel_E = vel_E;
    this->vel_D = vel_D;
}
