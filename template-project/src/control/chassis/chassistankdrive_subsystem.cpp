#include "chassistankdrive_subsystem.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

namespace control::chassis{

ChassisTankSubsystem::ChassisTankSubsystem(tap::Drivers* drivers, tap::motor::MotorId leftFrontMotorID, 
                        tap::motor::MotorId rightFrontMotorID, tap::motor::MotorId leftBackMotorID, 
                        tap::motor::MotorId rightBackMotorID, tap::can::CanBus canBus)
                        :tap::control::Subsystem(drivers),
                        leftFrontMotorPid(20.0f,0.0f,0.0f,5'000.0f,16'000.0f),
                        rightFrontMotorPid(20.0f,0.0f,0.0f,5'000.0f,16'000.0f),
                        leftBackMotorPid(20.0f,0.0f,0.0f,5'000.0f,16'000.0f),
                        rightBackMotorPid(20.0f,0.0f,0.0f,5'000.0f,16'000.0f),
                        leftFrontMotor(drivers,leftFrontMotorID,canBus,true,'LF'),
                        rightFrontMotor(drivers,rightFrontMotorID,canBus,false,'RF'),
                        leftBackMotor(drivers,leftBackMotorID,canBus,true,'LB'),
                        rightBackMotor(drivers,rightBackMotorID,canBus,false,'RB')
                        {
                            velocityPid[0] = leftFrontMotorPid;
                            velocityPid[1] = leftBackMotorPid;
                            velocityPid[2] = rightFrontMotorPid;
                            velocityPid[3] = rightBackMotorPid;

                            motors[0] = &leftFrontMotor;
                            motors[1] = &leftBackMotor;
                            motors[2] = &rightFrontMotor;
                            motors[3] = &rightBackMotor;
                        }


void ChassisTankSubsystem::initialize()
{
    for (int i = 0; i < 4; i++)
    {
        motors[i]->initialize();
    }
}

void ChassisTankSubsystem::setVelocityTankDrive(float left, float right)
{
    left = mpsToRpm(left);
    right = mpsToRpm(right);

    left = tap::algorithms::limitVal(left, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    right = tap::algorithms::limitVal(right, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);

    desiredOutput[0] = left;
    desiredOutput[1] = left;
    desiredOutput[2] = right;
    desiredOutput[3] = right;
}

void ChassisTankSubsystem::refresh()
{
     for (int i = 0; i < 4; i++)
    {
        velocityPid[i].update(desiredOutput[i] - motors[i]->getShaftRPM());
        motors[i]->setDesiredOutput(velocityPid[i].getValue());
    }
}

}





