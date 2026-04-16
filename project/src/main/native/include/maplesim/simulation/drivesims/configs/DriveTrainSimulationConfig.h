#pragma once
#include "maplesim/simulation/drivesims/GyroSimulation.h"
#include "maplesim/simulation/drivesims/SwerveModuleSimulation.h"

namespace maplesim {

class DriveTrainSimulationConfig {
   public:
    DriveTrainSimulationConfig& WithRobotMass(units::kilogram_t robotMass);

    DriveTrainSimulationConfig& WithBumperSize(units::meter_t bumperLengthX, units::meter_t bumperWidthY);

    DriveTrainSimulationConfig& WithTrackLengthTrackWidth(units::meter_t trackLengthX, units::meter_t trackWidthY);

    DriveTrainSimulationConfig& WithCustomModuleTranslations(std::vector<frc::Translation2d> moduleTranslations);

    DriveTrainSimulationConfig& WithSwerveModules(std::initializer_list<std::function<SwerveModuleSimulation()>> swerveModuleSimulationFactory);

    DriveTrainSimulationConfig& WithSwerveModule(std::function<SwerveModuleSimulation()> swerveModuleSimulationFactory);

    DriveTrainSimulationConfig& WithGyro(std::function<GyroSimulation()> gyroSimulationFactory);

    constexpr decltype(1_kg / 1_sq_m) GetDensity() const {
        return robotMass / (bumperLengthX * bumperWidthY);
    }

    units::meter_t TrackLengthX() const;
    units::meter_t TrackWidthY() const;

    constexpr units::meter_t DriveBaseRadius() const {
        return units::math::hypot(TrackLengthX(), TrackWidthY());
    }

    units::kilogram_t robotMass;
    units::meter_t bumperLengthX, bumperWidthY;
    std::vector<std::function<SwerveModuleSimulation()>> swerveModuleSimulationFactories;
    std::function<GyroSimulation()> gyroSimulationFactory;
    std::vector<frc::Translation2d> moduleTranslations;

    private:
    void CheckRobotMass() const;
    void CheckBumperSize() const;
    void CheckModuleTranslations() const;
};

}  // namespace maplesim
