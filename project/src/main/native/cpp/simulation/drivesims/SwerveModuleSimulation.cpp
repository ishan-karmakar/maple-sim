#include "maplesim/simulation/drivesims/SwerveModuleSimulation.h"
#include "maplesim/simulation/SimulatedArena.h"

using namespace maplesim;

SwerveModuleSimulation::SwerveModuleSimulation(SwerveModuleSimulationConfig config)
      : config{config}, steerMotorSim{config.steerMotorConfigs} {
    SimulatedBattery::AddElectricalAppliances([this] { return GetDriveMotorSupplyCurrent(); });
    steerMotorSim.UseSimpleDCMotorController();
}

Eigen::Vector2d SwerveModuleSimulation::UpdateSimulationSubTickGetModuleForce(Eigen::Vector2d moduleCurrentGroundVelocityWorldRelative,
                                                                              frc::Rotation2d robotFacing,
                                                                              units::newton_t gravityForceOnModule) {
    steerMotorSim.Update(SimulatedArena::GetSimulationDt());

    units::newton_t grippingForce = config.GetGrippingForce(gravityForceOnModule);
    frc::Rotation2d moduleWorldFacing = GetSteerAbsoluteFacing() + robotFacing;
    Eigen::Vector2d propellingForce = GetPropellingForce(grippingForce, moduleWorldFacing, moduleCurrentGroundVelocityWorldRelative);

    UpdateEncoderCaches();
    return propellingForce;
}

Eigen::Vector2d SwerveModuleSimulation::GetPropellingForce(units::newton_t grippingForce, frc::Rotation2d moduleWorldFacing,
                                                           Eigen::Vector2d moduleCurrentGroundVelocity) {
    units::newton_meter_t driveWheelTorque = GetDriveWheelTorque();
    units::newton_t propellingForce = driveWheelTorque / config.WHEEL_RADIUS;
    bool skidding = units::math::abs(propellingForce) > grippingForce;
    if (skidding)
        propellingForce = units::math::copysign(grippingForce, propellingForce);

    units::meters_per_second_t floorVelocityProjectionOnWheelDirection{
        moduleCurrentGroundVelocity.dot(moduleWorldFacing.ToMatrix().col(0))};

    driveWheelFinalSpeed = floorVelocityProjectionOnWheelDirection / config.WHEEL_RADIUS * 1_rad;

    if (skidding) {
        units::radians_per_second_t skiddingEquilibriumWheelSpeed = config.driveMotorConfigs.CalculateMechanismVelocity(
            config.driveMotorConfigs.CalculateCurrent(propellingForce * config.WHEEL_RADIUS), driveMotorAppliedVoltage);
        driveWheelFinalSpeed = driveWheelFinalSpeed / 2 + skiddingEquilibriumWheelSpeed / 2;
    }

    return Eigen::Vector2d{propellingForce * units::math::cos(moduleWorldFacing.Radians()),
                           propellingForce * units::math::sin(moduleWorldFacing.Radians())};
}

units::newton_meter_t SwerveModuleSimulation::GetDriveWheelTorque() {
    driveMotorAppliedVoltage = driveMotorController->UpdateControlSignal(driveWheelFinalPosition, driveWheelFinalSpeed,
                                                                         GetDriveEncoderUnGearedPosition(), GetDriveEncoderUnGearedSpeed());
    driveMotorAppliedVoltage = SimulatedBattery::Clamp(driveMotorAppliedVoltage);

    driveMotorStatorCurrent = config.driveMotorConfigs.CalculateCurrent(driveWheelFinalSpeed, driveMotorAppliedVoltage);

    units::newton_meter_t driveWheelTorque = config.driveMotorConfigs.CalculateTorque(driveMotorStatorCurrent);

    units::newton_meter_t driveWheelTorqueWithFriction = frc::ApplyDeadband(driveWheelTorque, config.driveMotorConfigs.friction,
                                                                            units::newton_meter_t{std::numeric_limits<double>::infinity()});
    return driveWheelTorqueWithFriction;
}

void SwerveModuleSimulation::UpdateEncoderCaches() {
    driveWheelFinalPosition = driveWheelFinalPosition + driveWheelFinalSpeed * SimulatedArena::GetSimulationDt();

    steerAbsolutePositionCache.push_back(GetSteerAbsoluteFacing());
    driveWheelFinalPositionCache.push_back(driveWheelFinalPosition);
}

std::vector<units::radian_t> SwerveModuleSimulation::GetCachedDriveEncoderUnGearedPositions() const {
    std::vector<units::radian_t> out;
    for (auto& v : driveWheelFinalPositionCache)
        out.push_back(v * config.DRIVE_GEAR_RATIO);
    return out;
}

std::vector<units::radian_t> SwerveModuleSimulation::GetCachedSteerRelativeEncoderPositions() const {
    std::vector<units::radian_t> out;
    for (auto& v : steerAbsolutePositionCache)
        out.push_back(v.Radians() * config.STEER_GEAR_RATIO + steerRelativeEncoderOffset);
    return out;
}
