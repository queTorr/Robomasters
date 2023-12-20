#include "agitator_subsystem.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "drivers.hpp"

namespace control::agitator{

AgitatorSubsystem::AgitatorSubsystem(src::Drivers* drivers)
    :tap::control::Subsystem(drivers),
    desiredAgitatorMotorOutput(0),
    agitatorMotorPid(10,0,0,0,16'000),
    agitatorMotor(drivers, AGITATOR_MOTOR_ID, CAN_BUS_MOTORS, false, "agitator motor"){}

void AgitatorSubsystem::initialize()
{
    agitatorMotor->initialize();
}

void AgitatorSubsystem::refresh(){}

void AgitatorSubsystem::setAgitatorVelocity(float input)
{
    desiredAgitatorMotorOutput = mpsToRpm(input);
  

    desiredAgitatorMotorOutput = 
    tap::algorithms::limitVal(desiredAgitatorMotorOutput, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
   
    agitatorMotorPid.update(desiredAgitatorMotorOutput - agitatorMotor->getShaftRPM())
    agitatorMotor->setDesiredOutput(agitatorMotorPid.getValue());
}




}