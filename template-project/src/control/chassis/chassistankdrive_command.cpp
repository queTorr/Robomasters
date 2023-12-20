
#include "control/chassis/chassistankdrive_command.hpp"
#include "control/chassis/chassistankdrive_subsystem.hpp"
#include "tap/algorithms/math_user_utils.hpp"


namespace control::chassis{

ChassisTankDriveCommand::ChassisTankDriveCommand(src::Drivers* drivers, ChassisTankSubsystem* chassisTank)
:drivers(drivers),chassisTank(chassisTank)
    {
        this->addSubsystemRequirement(chassisTank);
    }
bool ChassisTankDriveCommand::isReady() {return true;}
void ChassisTankDriveCommand::initialize() {}
void ChassisTankDriveCommand::execute() 
{
    float leftInput = drivers->controlOperatorInterface.getChassisTankLeftInput()*MAX_CHASSIS_SPEED_MPS;
    float rightInput = drivers->controlOperatorInterface.getChassisTankRightInput()*MAX_CHASSIS_SPEED_MPS;

    leftInput = tap::algorithms::limitVal(leftInput, -MAX_CHASSIS_SPEED_MPS, MAX_CHASSIS_SPEED_MPS );
    rightInput = tap::algorithms::limitVal(rightInput, -MAX_CHASSIS_SPEED_MPS, MAX_CHASSIS_SPEED_MPS );

    chassisTank->setVelocityTankDrive(leftInput, rightInput);
}

void ChassisTankDriveCommand::end(bool interrupted) 
{
    chassisTank->setVelocityTankDrive(0.0f,0.0f);
}

}
