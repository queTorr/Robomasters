#include "tap/motor/dji_motor.hpp"
#include "drivers_singleton.hpp"
#include "tap/control/subsystem.hpp"
#include "modm/math/geometry/angle.hpp"

namespace control::agitator{
class AgitatorSubsystem : public tap::control::Subsystem
{
    AgitatorSubsystem::AgitatorSubsystem(src::Drivers* drivers);

    void initialize() override;

    void refresh() override;

    void setAgitatorVelocity(float input);


    inline float mpsToRpm(float mps)
    {
        static constexpr float GEAR_RATIO = 36.0f;
        static constexpr float WHEEL_DIAMETER_M = 0.076f;
        static constexpr float WHEEL_CIRCUMFERANCE_M = M_PI * WHEEL_DIAMETER_M;
        static constexpr float SEC_PER_M = 60.0f;

        return (mps / WHEEL_CIRCUMFERANCE_M) * SEC_PER_M * GEAR_RATIO;
    }
    private:

    static constexpr tap::motor::MotorId AGITATOR_MOTOR_ID = tap::motor::MOTOR5;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

    float desiredAgitatorMotorOutput;
    modm::Pid<float> agitatorMotorPid;
    tap::motor::DjiMotor agitatorMotor;
}
}