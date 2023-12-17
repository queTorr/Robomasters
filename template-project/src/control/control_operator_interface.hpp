
#include "tap/util_macros.hpp"

namespace tap::communication::serial
{
class Remote;
}

namespace control
{
class ControlOperatorInterface
{
public:

    ControlOperatorInterface(tap::communication::serial::Remote &remote);

    virtual float getChassisTankLeftInput();
    virtual float getChassisTankRightInput();

private:
    tap::communication::serial::Remote &remote;
};
}  // namespace control