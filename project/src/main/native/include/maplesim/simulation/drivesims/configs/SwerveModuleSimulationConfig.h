#pragma once
#include <units/velocity.h>
#include "maplesim/motorsims/SimMotorConfigs.h"

namespace maplesim {

class SwerveModuleSimulationConfig {
   public:
    constexpr SwerveModuleSimulationConfig(frc::DCMotor driveMotorModel, frc::DCMotor steerMotorModel, double driveGearRatio,
                                           double steerGearRatio, units::volt_t driveFrictionVoltage, units::volt_t steerFrictionVoltage,
                                           units::meter_t wheelRadius, units::kilogram_square_meter_t steerRotationalInertia,
                                           double wheelsCoefficientOfFriction)
          : driveMotorConfigs{driveMotorModel, driveGearRatio, 0_kg_sq_m, driveFrictionVoltage},
            steerMotorConfigs{steerMotorModel, steerGearRatio, steerRotationalInertia, steerFrictionVoltage},
            DRIVE_GEAR_RATIO{driveGearRatio},
            STEER_GEAR_RATIO{steerGearRatio},
            WHEELS_COEFFICIENT_OF_FRICTION{wheelsCoefficientOfFriction},
            DRIVE_FRICTION_VOLTAGE{driveFrictionVoltage},
            WHEEL_RADIUS{wheelRadius} {}

    constexpr units::newton_t GetGrippingForceNewtons(units::newton_t gravityForceOnModuleNewtons) {
        return gravityForceOnModuleNewtons * WHEELS_COEFFICIENT_OF_FRICTION;
    }

    constexpr units::meters_per_second_t MaximumGroundSpeed() {
        return driveMotorConfigs.FreeSpinMechanismVelocity() * WHEEL_RADIUS / 1_tr;
    }

    constexpr units::newton_t GetTheoreticalPropellingForcePerModule(units::kilogram_t robotMass, int modulesCount,
                                                                     units::ampere_t statorCurrentLimit) {
        units::newton_t maxThrustNewtons = driveMotorConfigs.CalculateTorque(statorCurrentLimit) / WHEEL_RADIUS;
        units::newton_t maxGrippingNewtons = 9.8_mps_sq * robotMass / modulesCount * WHEELS_COEFFICIENT_OF_FRICTION;
        return units::math::min(maxThrustNewtons, maxGrippingNewtons);
    }

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
