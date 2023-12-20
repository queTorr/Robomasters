#include "standard.hpp"
#include "tap/util_macros.hpp"
#include "control/control_operator_interface.hpp"
#include "control/chassis/chassistankdrive_subsystem.hpp"
#include "control/chassis/chassistankdrive_command.hpp"
#include "drivers.hpp"
#include "drivers_singleton.hpp"

namespace control
{
Standard::Standard(src::Drivers* drivers):drivers(drivers),
    chassisTankSubsystem(drivers),
    chassisTankDriveCommand(drivers,&chassisTankSubsystem){}

void Standard::initializeSubsystems()
{
    chassisTankSubsystem.initialize();
}

void Standard::initSubsystemCommands()
{
    initializeSubsystems();
    registerStandardSubsystems();
    setDefaultStandardCommands();
}

void Standard::registerStandardSubsystems()
{
    drivers->commandScheduler.registerSubsystem(&chassisTankSubsystem);
}

void Standard::setDefaultStandardCommands()
{
    chassisTankSubsystem.setDefaultCommand(&chassisTankDriveCommand);
}

}