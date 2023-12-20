
#ifndef CHASSISTANKDRIVE_COMMAND_HPP_
#define CHASSISTANKDRIVE_COMMAND_HPP_

#include "tap/motor/dji_motor.hpp"
#include "drivers_singleton.hpp"
#include "tap/control/subsystem.hpp"
#include "modm/math/geometry/angle.hpp"

namespace control::chassis{

class ChassisTankSubsystem: public tap::control::Subsystem
{
    public:

    static constexpr float MAX_WHEELSPEED_RPM = 7000;

    ChassisTankSubsystem(src::Drivers* drivers);

    void initialize() override;

    void refresh() override;

    void setVelocityTankDrive(float left, float right);


    inline float mpsToRpm(float mps)
    {
        static constexpr float GEAR_RATIO = 19.0f;
        static constexpr float WHEEL_DIAMETER_M = 0.076f;
        static constexpr float WHEEL_CIRCUMFERANCE_M = M_PI * WHEEL_DIAMETER_M;
        static constexpr float SEC_PER_M = 60.0f;

        return (mps / WHEEL_CIRCUMFERANCE_M) * SEC_PER_M * GEAR_RATIO;
    }

    private:

    static constexpr tap::motor::MotorId LEFT_FRONT_MOTOR_ID = tap::motor::MOTOR3;
    static constexpr tap::motor::MotorId LEFT_BACK_MOTOR_ID = tap::motor::MOTOR1;
    static constexpr tap::motor::MotorId RIGHT_FRONT_MOTOR_ID = tap::motor::MOTOR2;
    static constexpr tap::motor::MotorId RIGHT_BACK_MOTOR_ID = tap::motor::MOTOR4;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

    src::Drivers* drivers;

    float desiredOutput[4];
    float leftFrontMotorOutput;
    float leftBackMotorOutput;
    float rightFrontMotorOutput;
    float rightBackMotorOutput;

    modm::Pid<float> velocityPid[4];
    modm::Pid<float> leftFrontMotorPid;
    modm::Pid<float> rightFrontMotorPid;
    modm::Pid<float> leftBackMotorPid;
    modm::Pid<float> rightBackMotorPid;
    
    tap::motor::DjiMotor* motors[4];
    tap::motor::DjiMotor leftFrontMotor;
    tap::motor::DjiMotor rightFrontMotor;
    tap::motor::DjiMotor leftBackMotor;
    tap::motor::DjiMotor rightBackMotor;

};

}//namespace

#endif