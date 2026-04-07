#include "maplesim/motorsims/SimMotorState.h"
#include <frc/MathUtil.h>
#include <units/math.h>

using namespace maplesim;

void SimMotorState::Step(units::newton_meter_t finalElectricTorque, units::newton_meter_t finalFrictionTorque,
                         units::kilogram_square_meter_t loadMOI, units::second_t dt) {
    mechanismAngularVelocity = 1_rad * finalElectricTorque / loadMOI * dt;

    units::radians_per_second_t deltaAngularVelocityDueToFriction =
        1_rad * units::math::copysign(finalFrictionTorque, -mechanismAngularVelocity) / loadMOI * dt;

    mechanismAngularVelocity = frc::ApplyDeadband(mechanismAngularVelocity + deltaAngularVelocityDueToFriction,
                                                  units::math::abs(deltaAngularVelocityDueToFriction), mechanismAngularVelocity);

    mechanismAngularPosition += mechanismAngularVelocity * dt;
}
