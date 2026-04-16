#pragma once
#include <deque>
#include <frc/kinematics/SwerveModuleState.h>
#include "maplesim/motorsims/MapleMotorSim.h"
#include "maplesim/simulation/drivesims/configs/SwerveModuleSimulationConfig.h"

namespace maplesim {

class SwerveModuleSimulation {
   public:
    SwerveModuleSimulation(SwerveModuleSimulationConfig config);

    constexpr const SimMotorConfigs& GetDriveMotorConfigs() const { return config.driveMotorConfigs; }

    constexpr const SimMotorConfigs& GetSteerMotorConfigs() const { return config.steerMotorConfigs; }

    template <typename T>
    inline T& UseDriveMotorController(std::unique_ptr<T> driveMotorController) {
        this->driveMotorController = std::move(driveMotorController);
        return *driveMotorController;
    }

    inline GenericMotorController& UseGenericMotorControllerForDrive() {
        return UseDriveMotorController(std::make_unique<GenericMotorController>(config.driveMotorConfigs.motor));
    }

    template <typename T>
    inline T& UseSteerMotorController(std::unique_ptr<T> steerMotorController) {
        return steerMotorSim.UseMotorController(steerMotorController);
    }

    inline GenericMotorController& UseGenericControllerForSteer() { return steerMotorSim.UseSimpleDCMotorController(); }

    Eigen::Vector2d UpdateSimulationSubTickGetModuleForce(Eigen::Vector2d moduleCurrentGroundVelocityWorldRelative,
                                                          frc::Rotation2d robotFacing, units::newton_t gravityForceOnModule);

    Eigen::Vector2d GetPropellingForce(units::newton_t grippingForce, frc::Rotation2d moduleWorldFacing,
                                       Eigen::Vector2d moduleCurrentGroundVelocity);

    units::newton_meter_t GetDriveWheelTorque();

    constexpr frc::SwerveModuleState GetCurrentState() const {
        return frc::SwerveModuleState{GetDriveWheelFinalSpeed() * config.WHEEL_RADIUS / 1_tr, GetSteerAbsoluteFacing()};
    }

    constexpr units::volt_t GetDriveMotorAppliedVoltage() const { return driveMotorAppliedVoltage; }

    constexpr units::volt_t GetSteerMotorAppliedVoltage() const { return steerMotorSim.GetAppliedVoltage(); }

    inline units::ampere_t GetDriveMotorSupplyCurrent() const {
        return GetDriveMotorStatorCurrent() * driveMotorAppliedVoltage / SimulatedBattery::GetBatteryVoltage();
    }

    constexpr units::ampere_t GetDriveMotorStatorCurrent() const { return driveMotorStatorCurrent; }

    inline units::ampere_t GetSteerMotorSupplyCurrent() const { return steerMotorSim.GetSupplyCurrent(); }

    constexpr units::ampere_t GetSteerMotorStatorCurrent() const { return steerMotorSim.GetStatorCurrent(); }

    constexpr units::radian_t GetDriveEncoderUnGearedPosition() const { return GetDriveWheelFinalPosition() * config.DRIVE_GEAR_RATIO; }

    constexpr units::radian_t GetDriveWheelFinalPosition() const { return driveWheelFinalPosition; }

    constexpr units::radians_per_second_t GetDriveEncoderUnGearedSpeed() const {
        return GetDriveWheelFinalSpeed() * config.DRIVE_GEAR_RATIO;
    }

    constexpr units::radians_per_second_t GetDriveWheelFinalSpeed() const { return driveWheelFinalSpeed; }

    constexpr units::radian_t GetSteerRelativeEncoderPosition() const {
        return GetSteerAbsoluteFacing().Radians() * config.STEER_GEAR_RATIO + steerRelativeEncoderOffset;
    }

    constexpr units::radians_per_second_t GetSteerRelativeEncoderVelocity() const {
        return GetSteerAbsoluteEncoderSpeed() * config.STEER_GEAR_RATIO;
    }

    constexpr frc::Rotation2d GetSteerAbsoluteFacing() const { return frc::Rotation2d{GetSteerAbsoluteAngle()}; }

    constexpr units::radian_t GetSteerAbsoluteAngle() const { return steerMotorSim.GetAngularPosition(); }

    constexpr units::radians_per_second_t GetSteerAbsoluteEncoderSpeed() const { return steerMotorSim.GetVelocity(); }

    std::vector<units::radian_t> GetCachedDriveEncoderUnGearedPositions() const;
    inline std::vector<units::radian_t> GetCachedDriveWheelFinalPositions() const {
        return std::vector<units::radian_t>{driveWheelFinalPositionCache.begin(), driveWheelFinalPositionCache.end()};
    }

    std::vector<units::radian_t> GetCachedSteerRelativeEncoderPositions() const;
    inline std::vector<frc::Rotation2d> GetCachedSteerAbsolutePositions() const {
        return std::vector<frc::Rotation2d>{steerAbsolutePositionCache.begin(), steerAbsolutePositionCache.end()};
    }

    SwerveModuleSimulationConfig config;

   protected:
    constexpr frc::SwerveModuleState GetFreeSpinState() const {
        return frc::SwerveModuleState{
            config.driveMotorConfigs.CalculateMechanismVelocity(
                config.driveMotorConfigs.CalculateCurrent(config.driveMotorConfigs.friction), driveMotorAppliedVoltage) *
                config.WHEEL_RADIUS / 1_rad,
            GetSteerAbsoluteFacing()};
    }

   private:
    void UpdateEncoderCaches();

    MapleMotorSim steerMotorSim{config.steerMotorConfigs};

    units::volt_t driveMotorAppliedVoltage;
    units::ampere_t driveMotorStatorCurrent;
    units::radian_t driveWheelFinalPosition;
    units::radians_per_second_t driveWheelFinalSpeed;

    std::unique_ptr<SimulatedMotorController> driveMotorController =
        std::make_unique<GenericMotorController>(config.driveMotorConfigs.motor);

    // TODO: Make random
    units::radian_t steerRelativeEncoderOffset = 0_rad;
    std::deque<units::radian_t> driveWheelFinalPositionCache;
    std::deque<frc::Rotation2d> steerAbsolutePositionCache;
};

}  // namespace maplesim
