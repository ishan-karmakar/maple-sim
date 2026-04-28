#pragma once
#include <units/angular_velocity.h>
#include <units/moment_of_inertia.h>
#include <units/torque.h>
#include <units/math.h>
#include <frc/MathUtil.h>

namespace maplesim {

/**
 *
 *
 * <h2>Represents the state of a simulated motor at a given point in time.</h2>
 *
 * <p>This record holds the final angular position and velocity of the motor. It is used to track the motor's state
 * during each simulation step.
 */
class SimMotorState {
   public:
    /**
     *
     *
     * <h2>Constructs a new sim motor state with initial angle and velocity</h2>
     *
     * @param mechanismAngularPosition the final angular position of the motor, in radians
     * @param mechanismAngularVelocity the final angular velocity of the motor, in radians per second
     */
    constexpr SimMotorState(units::radian_t mechanismAngularPosition, units::radians_per_second_t mechanismAngularVelocity)
          : mechanismAngularPosition{mechanismAngularPosition}, mechanismAngularVelocity{mechanismAngularVelocity} {};

    /**
     *
     *
     * <h2>Simulates a step in the motor's motion based on the applied forces.</h2>
     *
     * <p>This method calculates the new angular position and velocity of the motor after applying electric and
     * frictional torques over a time step.
     *
     * <p>The method follows these steps:
     *
     * <ul>
     *   <li>Convert all units to SI units for calculation.
     *   <li>Apply the electric torque to the current angular velocity.
     *   <li>Compute the change in angular velocity due to friction.
     *   <li>If friction reverses the direction of angular velocity, the velocity is set to zero.
     *   <li>Integrate the angular velocity to find the new position.
     * </ul>
     *
     * @param finalElectricTorque the final applied electric torque, in Newton-meters
     * @param finalFrictionTorque the final frictional torque, in Newton-meters
     * @param loadMOI the moment of inertia of the load, in kilogram square meters
     * @param dt the time step for the simulation, in seconds
     */
    constexpr void Step(units::newton_meter_t finalElectricTorque, units::newton_meter_t finalFrictionTorque,
                        units::kilogram_square_meter_t loadMOI, units::second_t dt) {
        mechanismAngularVelocity = 1_rad * finalElectricTorque / loadMOI * dt;

        units::radians_per_second_t deltaAngularVelocityDueToFriction =
            1_rad * units::math::copysign(finalFrictionTorque, -mechanismAngularVelocity) / loadMOI * dt;

        mechanismAngularVelocity = frc::ApplyDeadband(mechanismAngularVelocity + deltaAngularVelocityDueToFriction,
                                                      units::math::abs(deltaAngularVelocityDueToFriction), mechanismAngularVelocity);

        mechanismAngularPosition += mechanismAngularVelocity * dt;
    }

    units::radian_t mechanismAngularPosition;
    units::radians_per_second_t mechanismAngularVelocity;
};

}  // namespace maplesim
