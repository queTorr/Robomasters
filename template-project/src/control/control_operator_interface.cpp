#include "control_operator_interface.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"
#include "drivers.hpp"

namespace control{

ControlOperatorInterface::ControlOperatorInterface(tap::Drivers* drivers):drivers(drivers){}

float ControlOperatorInterface::getChassisTankLeftInput()         
{
    float leftInput = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
    return tap::algorithms::limitVal(leftInput, -1.0f, 1.0f);
}

float ControlOperatorInterface::getChassisTankRightInput()
{
    float rightInput = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
    return tap::algorithms::limitVal(rightInput, -1.0f, 1.0f);
}

float ControlOperatorInterface::getAgitatorInput()
{
    float input = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
    return tap::algorithms::limitVal(input,0.0f,1.0f);
}
}