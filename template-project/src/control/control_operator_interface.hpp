#pragma once
#ifndef CONTROL_OPERATOR_INTERFACE_HPP_
#define CONTROL_OPERATOR_INTERFACE_HPP_
#include "tap/util_macros.hpp"
#include "tap/drivers.hpp"
#include "tap/communication/serial/remote.hpp"

namespace control
{
class ControlOperatorInterface
{
public:

    ControlOperatorInterface(tap::Drivers* drivers);

    virtual float getChassisTankLeftInput();
    virtual float getChassisTankRightInput();
    virtual float getAgitatorInput();

private:
    tap::Drivers* drivers;
    //tap::communication::serial::Remote* remote;
};
}  // namespace control

#endif