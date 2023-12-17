#include "control_operator_interface.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"

namespace control{

ControlOperatorInterface::ControlOperatorInterface(tap::communication::serial::Remote& remote):remote(remote){}

float ControlOperatorInterface::getChassisTankLeftInput()         
{
    float leftInput = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
    return tap::algorithms::limitVal(leftInput, -1.0f, 1.0f);
}

float ControlOperatorInterface::getChassisTankRightInput()
{
    float rightInput = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
    return tap::algorithms::limitVal(rightInput, -1.0f, 1.0f);
}

}