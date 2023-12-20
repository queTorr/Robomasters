#pragma once

#include "control/chassis/chassistankdrive_subsystem.hpp"
#include "control/chassis/chassistankdrive_command.hpp"


namespace control{

class Standard
{
    public:
    
    Standard(src::Drivers* drivers);

    void initSubsystemCommands();

    private:   

    void initializeSubsystems();
    void registerStandardSubsystems();
    void setDefaultStandardCommands();
    void startStandardCommands();
    void registerStandardIoMappings();

    src::Drivers* drivers;
    
    chassis::ChassisTankSubsystem chassisTankSubsystem;
    chassis::ChassisTankDriveCommand chassisTankDriveCommand;
};
}
