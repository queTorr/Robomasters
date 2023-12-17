

#include "tap/util_macros.hpp"
#include "modm/math/filter/pid.hpp"
#include <cmath>
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "drivers_singleton.hpp"
#include "tap/control/subsystem.hpp"
#include "modm/math/geometry/angle.hpp"

namespace control::chassis{

class ChassisTankSubsystem: public tap::control::Subsystem
{
    public:

    static constexpr float MAX_WHEELSPEED_RPM = 7000;

    ChassisTankSubsystem(tap::Drivers* drivers, tap::motor::MotorId leftFrontMotorID, 
                        tap::motor::MotorId rightFrontMotorID, tap::motor::MotorId leftBackMotorID, 
                        tap::motor::MotorId rightBackMotorID, tap::can::CanBus canBus);

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

    float desiredOutput[4];

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
