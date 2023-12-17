
#include "chassistankdrive_command.hpp"
#include "tap/algorithms/math_user_utils.hpp"


namespace control::chassis{

ChassisTankDriveCommand::ChassisTankDriveCommand(ChassisTankSubsystem* chassisTank, ControlOperatorInterface* operatorInterface)
:chassisTank(chassisTank), operatorInterface(operatorInterface)
    {
        this->addSubsystemRequirement(chassisTank);
    }

bool ChassisTankDriveCommand::isReady() {}
void ChassisTankDriveCommand::initialize() {}
void ChassisTankDriveCommand::execute() 
{
    float leftInput = operatorInterface->getChassisTankLeftInput()*MAX_CHASSIS_SPEED_MPS;
    float rightInput = operatorInterface->getChassisTankRightInput()*MAX_CHASSIS_SPEED_MPS;

    leftInput = tap::algorithms::limitVal(leftInput, -MAX_CHASSIS_SPEED_MPS, MAX_CHASSIS_SPEED_MPS );
    rightInput = tap::algorithms::limitVal(rightInput, -MAX_CHASSIS_SPEED_MPS, MAX_CHASSIS_SPEED_MPS );

    chassisTank->setVelocityTankDrive(leftInput, rightInput);
}

void ChassisTankDriveCommand::end(bool interrupted) 
{
    chassisTank->setVelocityTankDrive(0.0f,0.0f);
}

bool ChassisTankDriveCommand::isFinished() const {return true;}
    
}