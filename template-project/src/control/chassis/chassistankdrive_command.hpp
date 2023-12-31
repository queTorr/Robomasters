#ifndef CHASSISTANKDRIVE_SUBSYSTEM_HPP_
#define CHASSISTANKDRIVE_SUBSYSTEM_HPP_

#include "tap/control/command.hpp"
#include "chassistankdrive_subsystem.hpp"
#include "drivers.hpp"

namespace control
{
class ControlOperatorInterface;
}


namespace control::chassis{

class ChassisTankDriveCommand : public tap::control::Command
{
    public:
    static constexpr float MAX_CHASSIS_SPEED_MPS = 3.0f;

    ChassisTankDriveCommand(src::Drivers* drivers, ChassisTankSubsystem* chassisTank);
   
    const char *getName() const override { return "Chassis tank drive"; }
    bool isReady() override;
    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override {return false;}

    private:

    src::Drivers* drivers;
    ChassisTankSubsystem* chassisTank;

};

}//namespace:chassis

#endif
