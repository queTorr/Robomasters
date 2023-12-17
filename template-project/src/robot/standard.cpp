#include "standard.hpp"

#include "tap/util_macros.hpp"

#include "control/chassis/chassistankdrive_subsystem.hpp"
#include "control/chassis/chassistankdrive_command.hpp"

#include "drivers.hpp"

namespace control
{

Standard::Standard(tap::Drivers* drivers):drivers(drivers),
    chassisTankSubsystem(drivers, tap::motor::MOTOR1,
                        tap::motor::MOTOR2, tap::motor::MOTOR3,
                        tap::motor::MOTOR4, tap::can::CanBus::CAN_BUS1),
    chassisTankDriveCommand(chassisTankSubsystem,drivers.controlOperatorInterface){}

void Standard::initializeSubsystems()
{
    chassisTankSubsystem.initialize();
}

void Standard::registerSoldierSubsystems()
{
    drivers->commandScheduler.registerSubsystem(chassisTankSubsystem);
}

void Standard::setDefaultSoldierCommands()
{
    chassisTankSubsystem->setDefaultCommand(chassisTankDriveCommand);
}

void Robot::startSoldierCommands() {}

void Robot::registerSoldierIoMappings()
{
    // STEP 9 (Agitator Control): register HoldRepeatCommandMapping and HoldCommandMapping

    drivers.commandMapper.addMap(&rightSwitchUp);
    drivers.commandMapper.addMap(&leftMousePressed);
   
}

}