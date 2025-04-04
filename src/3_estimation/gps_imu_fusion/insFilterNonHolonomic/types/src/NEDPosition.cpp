#include <InsFilterNonHolonomicTypes.hpp>

using namespace InsFilterNonHolonomicTypes;

NEDPosition::NEDPosition(double pos_N, double pos_E, double pos_D)
{
    this->pos_N = pos_N;
    this->pos_E = pos_E;
    this->pos_D = pos_D;
}