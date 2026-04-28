#include "maplesim/simulation/drivesims/configs/BoundingCheck.h"
#include "maplesim/simulation/drivesims/configs/DriveTrainSimulationConfig.h"

using namespace maplesim;

DriveTrainSimulationConfig& DriveTrainSimulationConfig::WithRobotMass(units::kilogram_t robotMass) {
    this->robotMass = robotMass;
    CheckRobotMass();
    return *this;
}

DriveTrainSimulationConfig& DriveTrainSimulationConfig::WithBumperSize(units::meter_t bumperLengthX, units::meter_t bumperWidthY) {
    this->bumperLengthX = bumperLengthX;
    this->bumperWidthY = bumperWidthY;
    CheckBumperSize();
    return *this;
}

DriveTrainSimulationConfig& DriveTrainSimulationConfig::WithTrackLengthTrackWidth(units::meter_t trackLengthX, units::meter_t trackWidthY) {
    BoundingCheck::Check(trackLengthX, 0.2_m, 1.5_m, "track length", "meters");
    BoundingCheck::Check(trackWidthY, 0.2_m, 1.5_m, "track width", "meters");
    moduleTranslations = {frc::Translation2d{trackLengthX / 2, trackWidthY / 2}, frc::Translation2d{trackLengthX / 2, -trackWidthY / 2},
                          frc::Translation2d{-trackLengthX / 2, trackWidthY / 2}, frc::Translation2d{-trackLengthX / 2, -trackWidthY / 2}};
    return *this;
}

DriveTrainSimulationConfig& DriveTrainSimulationConfig::WithCustomModuleTranslations(std::vector<frc::Translation2d> moduleTranslations) {
    CheckModuleTranslations();
    this->moduleTranslations = moduleTranslations;
    return *this;
}

DriveTrainSimulationConfig& DriveTrainSimulationConfig::WithSwerveModules(
    std::initializer_list<std::function<SwerveModuleSimulation()>> swerveModuleSimulationFactory) {
    if (swerveModuleSimulationFactory.size() == 1)
        return WithSwerveModule(*swerveModuleSimulationFactory.begin());

    FRC_AssertMessage(swerveModuleSimulationFactory.size() == moduleTranslations.size(),
                      "Module simulation factories length must be 1 or 4, provided {}", swerveModuleSimulationFactory.size());

    swerveModuleSimulationFactories = swerveModuleSimulationFactory;
    return *this;
}

DriveTrainSimulationConfig& DriveTrainSimulationConfig::WithSwerveModule(
    std::function<SwerveModuleSimulation()> swerveModuleSimulationFactory) {
    swerveModuleSimulationFactories.resize(4);
    for (int i = 0; i < 4; i++)
        swerveModuleSimulationFactories.push_back(swerveModuleSimulationFactory);
    return *this;
}

DriveTrainSimulationConfig& DriveTrainSimulationConfig::WithGyro(std::function<GyroSimulation()> gyroSimulationFactory) {
    this->gyroSimulationFactory = gyroSimulationFactory;
    return *this;
}

units::meter_t DriveTrainSimulationConfig::TrackLengthX() const {
    FRC_AssertMessage(!moduleTranslations.empty(), "Module translations are empty");
    units::meter_t maxModuleX{std::numeric_limits<double>::min()};
    units::meter_t minModuleX{std::numeric_limits<double>::max()};
    for (const auto& trans : moduleTranslations) {
        maxModuleX = units::math::max(maxModuleX, trans.X());
        minModuleX = units::math::min(minModuleX, trans.X());
    }
    return maxModuleX - minModuleX;
}

units::meter_t DriveTrainSimulationConfig::TrackWidthY() const {
    FRC_AssertMessage(!moduleTranslations.empty(), "Module translations are empty");
    units::meter_t maxModuleY{std::numeric_limits<double>::min()};
    units::meter_t minModuleY{std::numeric_limits<double>::max()};
    for (const auto& trans : moduleTranslations) {
        maxModuleY = units::math::max(maxModuleY, trans.Y());
        minModuleY = units::math::min(minModuleY, trans.Y());
    }
    return maxModuleY - minModuleY;
}

void DriveTrainSimulationConfig::CheckRobotMass() const {
    BoundingCheck::Check(bumperLengthX, 0.2_m, 1.5_m, "bumper length", "meters");
    BoundingCheck::Check(bumperWidthY, 0.2_m, 1.5_m, "bumper width", "meters");
}

void DriveTrainSimulationConfig::CheckModuleTranslations() const {
    for (size_t i = 0; i < moduleTranslations.size(); i++)
        BoundingCheck::Check(moduleTranslations[i].Norm(), 0.2_m, 1.2_m, "module number " + std::to_string(i) + " translation magnitude",
                             "meters");
}
