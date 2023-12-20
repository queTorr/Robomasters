#include "tap/control/command.hpp"
#include "agitator_subsystem.hpp"
#include "drivers.hpp"

namespace control
{
class ControlOperatorInterface;
}

namespace control::agitator{

class ChassisTankDriveCommand : public tap::control::Command
{
    public:
    static constexpr float MAX_CHASSIS_SPEED_MPS = 3.0f;

    ChassisTankDriveCommand(src::Drivers* drivers, AgitatorSubsystem* agitatorSubsystem);
   
    const char *getName() const override { return "agitator command"; }
    bool isReady() override;
    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override {return false;}

    private:

    src::Drivers* drivers;
    AgitatorSubsystem* agitatorSubsystem;

};

}//namespace:chassis