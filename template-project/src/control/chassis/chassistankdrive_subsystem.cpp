#include "chassistankdrive_subsystem.hpp"
#include "chassistankdrive_command.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "drivers.hpp"

namespace control::chassis{

ChassisTankSubsystem::ChassisTankSubsystem(src::Drivers* drivers)
                        :tap::control::Subsystem(drivers),drivers(drivers),
                        leftFrontMotorOutput(0),
                        leftBackMotorOutput(0),
                        rightFrontMotorOutput(0),
                        rightBackMotorOutput(0),
                        leftFrontMotorPid(10, 0, 0, 0, 16'000),
                        leftBackMotorPid(10, 0, 0, 0, 16'000),
                        rightFrontMotorPid(10, 0, 0, 0, 16'000),
                        rightBackMotorPid(10, 0, 0, 0, 16'000),
                        leftFrontMotor(drivers,LEFT_FRONT_MOTOR_ID,CAN_BUS_MOTORS,true,"LF"),
                        leftBackMotor(drivers,LEFT_BACK_MOTOR_ID,CAN_BUS_MOTORS,false,"RF"),
                        rightFrontMotor(drivers,RIGHT_FRONT_MOTOR_ID,CAN_BUS_MOTORS,true,"LB"),
                        rightBackMotor(drivers,RIGHT_BACK_MOTOR_ID,CAN_BUS_MOTORS,false,"RB")
                        {
                            desiredOutput[0] = leftFrontMotorOutput;
                            desiredOutput[1] = leftBackMotorOutput;
                            desiredOutput[2] = rightFrontMotorOutput;
                            desiredOutput[3] = rightBackMotorOutput;

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

     for (int i = 0; i < 4; i++)
    {
        velocityPid[i].update(desiredOutput[i] - motors[i]->getShaftRPM());
        motors[i]->setDesiredOutput(velocityPid[i].getValue());
    }
}

void ChassisTankSubsystem::refresh()
{
   
}

}





