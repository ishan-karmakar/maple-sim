#pragma once
#include <frc/system/plant/DCMotor.h>
#include <units/moment_of_inertia.h>

namespace maplesim {

/**
 *
 *
 * <h1>Stores the configurations of the motor.</h1>
 *
 * <p>This class encapsulates the various configuration parameters required to simulate and control a motor in a system.
 * The configurations include:
 *
 * <ul>
 *   <li><strong>motor:</strong> The motor model used in the simulation (e.g., Falcon 500, NEO).
 *   <li><strong>gearing:</strong> The gear ratio between the motor and the load, affecting the output torque and speed.
 *   <li><strong>loadMOI:</strong> The moment of inertia (MOI) of the load connected to the motor, which determines the
 *       resistance to changes in rotational speed.
 *   <li><strong>friction:</strong> The torque friction characteristics applied to the motor's simulation, representing
 *       real-world losses.
 *   <li><strong>positionVoltageController:</strong> PID controller for controlling the motor's position via voltage.
 *   <li><strong>velocityVoltageController:</strong> PID controller for controlling the motor's velocity via voltage.
 *   <li><strong>positionCurrentController:</strong> PID controller for controlling the motor's position via current.
 *   <li><strong>velocityCurrentController:</strong> PID controller for controlling the motor's velocity via current.
 *   <li><strong>feedforward:</strong> A feedforward controller used to compensate for the desired motor behavior based
 *       on input speeds.
 *   <li><strong>forwardHardwareLimit:</strong> The forward limit for motor rotation, specified in angle units.
 *   <li><strong>reverseHardwareLimit:</strong> The reverse limit for motor rotation, specified in angle units.
 *   <li><strong>currentLimit:</strong> The current limit applied to the motor to protect it from overcurrent
 *       conditions.
 * </ul>
 */
class SimMotorConfigs {
   public:
    /**
     *
     *
     * <h2>Constructs a simulated motor configuration.</h2>
     *
     * <p>This constructor initializes a {@link SimMotorConfigs} object with the necessary parameters for motor
     * simulation, including the motor model, gearing ratio, load moment of inertia, and friction characteristics.
     *
     * @param motor the motor model to be used in the simulation (e.g., Falcon 500, NEO).
     * @param gearing the gear ratio between the motor and the load, affecting torque and speed output.
     * @param loadMOI the moment of inertia of the load connected to the motor, representing rotational resistance.
     * @param frictionVoltage the voltage applied to simulate frictional torque losses in the motor.
     */
    constexpr SimMotorConfigs(frc::DCMotor motor, double gearing, units::kilogram_square_meter_t loadMOI, units::volt_t frictionVoltage)
          : motor{motor}, gearing{gearing}, loadMOI{loadMOI}, friction{motor.Torque(motor.Current(0_rad_per_s, frictionVoltage))} {}

    /**
     *
     *
     * <h2>Calculates the voltage of the motor.</h2>
     *
     * <p>This method uses the {@link DCMotor} model to find the voltage for a given current and angular velocity.
     *
     * @see DCMotor#getVoltage(double, double) for the underlying implementation.
     * @param current the current flowing through the motor
     * @param mechanismVelocity the final angular velocity of the mechanism
     * @return the voltage required for the motor to achieve the specified current and angular velocity
     */
    constexpr units::volt_t CalculateVoltage(units::ampere_t current, units::radians_per_second_t mechanismVelocity) const {
        return motor.Voltage(motor.Torque(current), mechanismVelocity * gearing);
    }

    /**
     *
     *
     * <h2>Calculates the velocity of the motor.</h2>
     *
     * <p>This method uses the {@link DCMotor} model to find the angular velocity for a given current and voltage.
     *
     * @see DCMotor#getSpeed(double, double) for the underlying implementation.
     * @param current the current flowing through the motor.
     * @param voltage the voltage applied to the motor.
     * @return the final angular velocity of the mechanism.
     */
    constexpr units::radians_per_second_t CalculateMechanismVelocity(units::ampere_t current, units::volt_t voltage) const {
        return motor.Speed(motor.Torque(current), voltage) / gearing;
    }

    /**
     *
     *
     * <h2>Calculates the current of the motor.</h2>
     *
     * <p>This method uses the {@link DCMotor} model to find the current for a given angular velocity and voltage.
     *
     * @see DCMotor#getCurrent(double, double) for the underlying implementation.
     * @param mechanismVelocity the final angular velocity of the mechanism.
     * @param voltage the voltage applied to the moto.
     * @return the current drawn by the motor.
     */
    constexpr units::ampere_t CalculateCurrent(units::radians_per_second_t mechanismVelocity, units::volt_t voltage) const {
        return motor.Current(mechanismVelocity * gearing, voltage);
    }

    /**
     *
     *
     * <h2>Calculates the current based on the motor's torque.</h2>
     *
     * <p>This method uses the {@link DCMotor} model to find the current required for a given torque.
     *
     * @see DCMotor#getCurrent(double) for the underlying implementation.
     * @param torque the final torque generated by the motor on the mechanism.
     * @return the current required to produce the specified torque.
     */
    constexpr units::ampere_t CalculateCurrent(units::newton_meter_t torque) const { return motor.Current(torque / gearing); }

    /**
     *
     *
     * <h2>Calculates the torque based on the motor's current.</h2>
     *
     * <p>This method uses the {@link DCMotor} model to find the torque generated by a given current.
     *
     * @see DCMotor#getTorque(double) for the underlying implementation.
     * @param current the current flowing through the motor.
     * @return the torque generated by the motor.
     */
    constexpr units::newton_meter_t CalculateTorque(units::ampere_t current) const { return motor.Torque(current) * gearing; }

    /**
     *
     *
     * <h2>Configures the hard limits for the motor.</h2>
     *
     * <p>This method sets the hardware limits for the motor's movement. When either the forward or reverse limit is
     * reached, the motor will be physically restricted from moving beyond that point, based on the motor's hardware
     * constraints.
     *
     * @param forwardLimit the forward hardware limit angle, beyond which the motor cannot move
     * @param reverseLimit the reverse hardware limit angle, beyond which the motor cannot move
     * @return this instance for method chaining
     */
    constexpr SimMotorConfigs& WithHardLimits(units::radian_t forwardLimit, units::radian_t reverseLimit) {
        forwardHardwareLimit = forwardLimit;
        reverseHardwareLimit = reverseLimit;
        return *this;
    }

    constexpr units::radians_per_second_t FreeSpinMechanismVelocity() const { return motor.freeSpeed / gearing; }

    constexpr units::ampere_t FreeSpinCurrent() const { return motor.freeCurrent; }

    constexpr units::ampere_t StallCurrent() const { return motor.stallCurrent; }

    constexpr units::newton_meter_t StallTorque() const { return motor.stallTorque; }

    constexpr units::volt_t NominalVoltage() const { return motor.nominalVoltage; }

    frc::DCMotor motor;
    double gearing;
    units::kilogram_square_meter_t loadMOI;
    units::newton_meter_t friction;

    units::radian_t forwardHardwareLimit{std::numeric_limits<double>::infinity()};
    units::radian_t reverseHardwareLimit{-std::numeric_limits<double>::infinity()};
};

}  // namespace maplesim
