
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"
#include "control/control_operator_interface.hpp"
#include "chassistankdrive_subsystem.hpp"

namespace control::chassis{

class ChassisTankDriveCommand : public tap::control::Command
{
    public:

    ChassisTankDriveCommand(ChassisTankSubsystem* chassisTank, ControlOperatorInterface* operatorInterface)
    :chassisTank(chassisTank), operatorInterface(operatorInterface)
    {
        this->addSubsystemRequirement(chassisTank);
    }



    bool isReady() override;
    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;

    private:

    ChassisTankSubsystem* chassisTank;
    ControlOperatorInterface* operatorInterface;
};


}