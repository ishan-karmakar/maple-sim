#pragma once
#include <units/velocity.h>
#include "maplesim/motorsims/SimMotorConfigs.h"

namespace maplesim {

class SwerveModuleSimulationConfig {
   public:
    /**
     *
     *
     * <h2>Constructs a Configuration for Swerve Module Simulation.</h2>
     *
     * <p>If you are using {@link SimulatedArena#overrideSimulationTimings(Time, int)} to use custom timings, you must
     * call the method before constructing any swerve module simulations using this constructor.
     *
     * @param driveMotorModel the model of the driving motor
     * @param steerMotorModel; the model of the steering motor
     * @param driveGearRatio the gear ratio for the driving motor, >1 is reduction
     * @param steerGearRatio the gear ratio for the steering motor, >1 is reduction
     * @param driveFrictionVoltage the measured minimum amount of voltage that can turn the driving rotter
     * @param steerFrictionVoltage the measured minimum amount of voltage that can turn the steering rotter
     * @param wheelRadius the radius of the wheels.
     * @param steerRotationalInertia the rotational inertia of the entire steering mechanism
     * @param wheelsCoefficientOfFriction the <a
     *     href='https://simple.wikipedia.org/wiki/Coefficient_of_friction#:~:text=A%20coefficient%20of%20friction%20is%20a%20value%20that%20shows%20the'>coefficient
     *     of friction</a> of the tires, normally around 1.2 {@link Units#inchesToMeters(double)}.
     */
    SwerveModuleSimulationConfig(frc::DCMotor driveMotorModel, frc::DCMotor steerMotorModel, double driveGearRatio, double steerGearRatio,
                                 units::volt_t driveFrictionVoltage, units::volt_t steerFrictionVoltage, units::meter_t wheelRadius,
                                 units::kilogram_square_meter_t steerRotationalInertia, double wheelsCoefficientOfFriction);

    constexpr units::newton_t GetGrippingForce(units::newton_t gravityForceOnModule) {
        return gravityForceOnModule * WHEELS_COEFFICIENT_OF_FRICTION;
    }

    /**
     *
     *
     * <h2>Obtains the theoretical speed that the module can achieve.</h2>
     *
     * @return the theoretical maximum ground speed that the module can achieve, in m/s
     */
    constexpr units::meters_per_second_t MaximumGroundSpeed() {
        return driveMotorConfigs.FreeSpinMechanismVelocity() * WHEEL_RADIUS / 1_tr;
    }

    /**
     *
     *
     * <h2>Obtains the theoretical maximum propelling force of ONE module.</h2>
     *
     * <p>Calculates the maximum propelling force with respect to the gripping force and the drive motor's torque under
     * its current limit.
     *
     * @param robotMass the mass of the robot
     * @param modulesCount the amount of modules on the robot, assumed to be sharing the gravity force equally
     * @return the maximum propelling force of EACH module
     */
    constexpr units::newton_t GetTheoreticalPropellingForcePerModule(units::kilogram_t robotMass, int modulesCount,
                                                                     units::ampere_t statorCurrentLimit) {
        units::newton_t maxThrustNewtons = driveMotorConfigs.CalculateTorque(statorCurrentLimit) / WHEEL_RADIUS;
        units::newton_t maxGrippingNewtons = 9.8_mps_sq * robotMass / modulesCount * WHEELS_COEFFICIENT_OF_FRICTION;
        return units::math::min(maxThrustNewtons, maxGrippingNewtons);
    }

    /**
     *
     *
     * <h2>Obtains the theatrical linear acceleration that the robot can achieve.</h2>
     *
     * <p>Calculates the maximum linear acceleration of a robot, with respect to its mass and
     * {@link #getTheoreticalPropellingForcePerModule(Mass, int, Current)}.
     *
     * @param robotMass the mass of the robot
     * @param modulesCount the amount of modules on the robot, assumed to be sharing the gravity force equally
     */
    constexpr units::meters_per_second_squared_t MaxAcceleration(units::kilogram_t robotMass, int modulesCount,
                                                                 units::ampere_t statorCurrentLimit) {
        return GetTheoreticalPropellingForcePerModule(robotMass, modulesCount, statorCurrentLimit) * modulesCount / robotMass;
    }

    SimMotorConfigs driveMotorConfigs, steerMotorConfigs;
    double DRIVE_GEAR_RATIO, STEER_GEAR_RATIO, WHEELS_COEFFICIENT_OF_FRICTION;
    units::volt_t DRIVE_FRICTION_VOLTAGE;
    units::meter_t WHEEL_RADIUS;
};

}  // namespace maplesim
