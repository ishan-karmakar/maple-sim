#include <frc/MathUtil.h>
#include "maplesim/simulation/drivesims/configs/BoundingCheck.h"
#include "maplesim/simulation/drivesims/configs/SwerveModuleSimulationConfig.h"

using namespace maplesim;

SwerveModuleSimulationConfig::SwerveModuleSimulationConfig(frc::DCMotor driveMotorModel, frc::DCMotor steerMotorModel,
                                                           double driveGearRatio, double steerGearRatio, units::volt_t driveFrictionVoltage,
                                                           units::volt_t steerFrictionVoltage, units::meter_t wheelRadius,
                                                           units::kilogram_square_meter_t steerRotationalInertia,
                                                           double wheelsCoefficientOfFriction)
      : driveMotorConfigs{driveMotorModel, driveGearRatio, 0_kg_sq_m, driveFrictionVoltage},
        steerMotorConfigs{steerMotorModel, steerGearRatio, steerRotationalInertia, steerFrictionVoltage},
        DRIVE_GEAR_RATIO{driveGearRatio},
        STEER_GEAR_RATIO{steerGearRatio},
        WHEELS_COEFFICIENT_OF_FRICTION{wheelsCoefficientOfFriction},
        DRIVE_FRICTION_VOLTAGE{driveFrictionVoltage},
        WHEEL_RADIUS{wheelRadius} {
    BoundingCheck::Check<double>(driveGearRatio, 4, 24, "drive gear ratio", "times reduction");
    BoundingCheck::Check<double>(steerGearRatio, 6, 50, "steer gear ratio", "times reduction");
    BoundingCheck::Check(driveFrictionVoltage, 0.01_V, 0.35_V, "drive friction voltage", "volts");
    BoundingCheck::Check(steerFrictionVoltage, 0.01_V, 0.6_V, "steer friction voltage", "volts");
    BoundingCheck::Check<units::inch_t>(wheelRadius, 1_in, 3.2_in, "drive wheel radius", "inches");
    BoundingCheck::Check(steerRotationalInertia, 0.005_kg_sq_m, 0.06_kg_sq_m, "steer rotation inertia", "kg * m^2");
    BoundingCheck::Check(wheelsCoefficientOfFriction, 0.6, 2.5, "tire coefficient of friction", "");
}
