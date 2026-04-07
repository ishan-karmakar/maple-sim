#pragma once
#include <units/angular_velocity.h>
#include <units/moment_of_inertia.h>
#include <units/torque.h>
#include <units/math.h>
#include <frc/MathUtil.h>

namespace maplesim {

class SimMotorState {
   public:
    units::radian_t mechanismAngularPosition;
    units::radians_per_second_t mechanismAngularVelocity;

    constexpr SimMotorState(units::radian_t mechanismAngularPosition, units::radians_per_second_t mechanismAngularVelocity)
          : mechanismAngularPosition{mechanismAngularPosition}, mechanismAngularVelocity{mechanismAngularVelocity} {};

    constexpr void Step(units::newton_meter_t finalElectricTorque, units::newton_meter_t finalFrictionTorque,
                        units::kilogram_square_meter_t loadMOI, units::second_t dt) {
        mechanismAngularVelocity = 1_rad * finalElectricTorque / loadMOI * dt;

        units::radians_per_second_t deltaAngularVelocityDueToFriction =
            1_rad * units::math::copysign(finalFrictionTorque, -mechanismAngularVelocity) / loadMOI * dt;

        mechanismAngularVelocity = frc::ApplyDeadband(mechanismAngularVelocity + deltaAngularVelocityDueToFriction,
                                                      units::math::abs(deltaAngularVelocityDueToFriction), mechanismAngularVelocity);

        mechanismAngularPosition += mechanismAngularVelocity * dt;
    }
};

}  // namespace maplesim
