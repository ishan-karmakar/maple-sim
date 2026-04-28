#pragma once
#include "maplesim/simulation/drivesims/GyroSimulation.h"
#include "maplesim/simulation/drivesims/SwerveModuleSimulation.h"
#include "maplesim/simulation/drivesims/COTS.h"

namespace maplesim {

/**
 *
 *
 * <h1>Stores the configurations for a swerve drive simulation.</h1>
 *
 * <p>This class is used to hold all the parameters necessary for simulating a swerve drivetrain, allowing for realistic
 * performance testing and evaluation.
 */
class DriveTrainSimulationConfig {
   public:
    /**
     *
     *
     * <h2>Ordinary Constructor</h2>
     *
     * <p>Creates an instance of {@link DriveTrainSimulationConfig} with specified parameters.
     *
     * @param robotMass the mass of the robot, including bumpers.
     * @param bumperLengthX the length of the bumper (distance from front to back).
     * @param bumperWidthY the width of the bumper (distance from left to right).
     * @param trackLengthX the distance between the front and rear wheels.
     * @param trackWidthY the distance between the left and right wheels.
     * @param swerveModuleSimulationFactory the factory that creates appropriate swerve module simulation for the
     *     drivetrain. You can specify one factory to apply the same configuration over all modules or specify four
     *     factories in the order (FL, FR, BL, BR).
     * @param gyroSimulationFactory the factory that creates appropriate gyro simulation for the drivetrain.
     */
    DriveTrainSimulationConfig(units::kilogram_t robotMass, units::meter_t bumperLengthX, units::meter_t bumperWidthY,
                               units::meter_t trackLengthX, units::meter_t trackWidthY,
                               std::function<GyroSimulation()> gyroSimulationFactory,
                               std::initializer_list<std::function<SwerveModuleSimulation()>> swerveModuleSimulationFactory);

    /**
     *
     *
     * <h2>Default Constructor.</h2>
     *
     * <p>Creates a {@link DriveTrainSimulationConfig} with all the data set to default values.
     *
     * <p>Though the config starts with default values, any configuration can be modified after creation.
     *
     * <p>The default configurations are:
     *
     * <ul>
     *   <li>Robot Mass of 45 kilograms.
     *   <li>Bumper Length of 0.76 meters.
     *   <li>Bumper Width of 0.76 meters.
     *   <li>Track Length of 0.52 meters.
     *   <li>Track Width of 0.52 meters.
     *   <li>Default swerve module simulations based on Falcon 500 motors.
     *   <li>Default gyro simulation using the Pigeon2 gyro.
     * </ul>
     *
     * @return a new instance of {@link DriveTrainSimulationConfig} with all configs set to default values.
     */
    inline static DriveTrainSimulationConfig Default() {
        return {45_kg, 0.76_m, 0.76_m, 0.52_m, 0.52_m, COTS::OfPigeon2(), COTS::OfMark4(frc::DCMotor::Falcon500())};
    }

    /**
     *
     *
     * <h2>Sets the robot mass.</h2>
     *
     * <p>Updates the mass of the robot in kilograms.
     *
     * @param robotMass the new mass of the robot.
     * @return the current instance of {@link DriveTrainSimulationConfig} for method chaining.
     */
    DriveTrainSimulationConfig& WithRobotMass(units::kilogram_t robotMass);

    /**
     *
     *
     * <h2>Sets the bumper size.</h2>
     *
     * <p>Updates the dimensions of the bumper.
     *
     * @param bumperLengthX the length of the bumper.
     * @param bumperWidthY the width of the bumper.
     * @return the current instance of {@link DriveTrainSimulationConfig} for method chaining.
     */
    DriveTrainSimulationConfig& WithBumperSize(units::meter_t bumperLengthX, units::meter_t bumperWidthY);

    /**
     *
     *
     * <h2>Sets the track length and width.</h2>
     *
     * <p>Updates the translations for the swerve modules based on the specified track length and track width.
     *
     * <p>For non-rectangular chassis configuration, use {@link #withCustomModuleTranslations(Translation2d[])} instead.
     *
     * @param trackLengthX the distance between the front and rear wheels.
     * @param trackWidthY the distance between the left and right wheels.
     * @return the current instance of {@link DriveTrainSimulationConfig} for method chaining.
     */
    DriveTrainSimulationConfig& WithTrackLengthTrackWidth(units::meter_t trackLengthX, units::meter_t trackWidthY);

    /**
     *
     *
     * <h2>Sets custom module translations.</h2>
     *
     * <p>Updates the translations of the swerve modules with user-defined values.
     *
     * <p>For ordinary rectangular modules configuration, use {@link #withTrackLengthTrackWidth(Distance, Distance)}
     * instead.
     *
     * @param moduleTranslations the custom translations for the swerve modules.
     * @return the current instance of {@link DriveTrainSimulationConfig} for method chaining.
     */
    DriveTrainSimulationConfig& WithCustomModuleTranslations(std::vector<frc::Translation2d> moduleTranslations);

    /**
     *
     *
     * <h2>Sets the swerve module simulation factory.</h2>
     *
     * <p>Updates the factory used to create swerve module simulations.
     *
     * @param swerveModuleSimulationFactory the new factory (or factories) for swerve module simulations. You can
     *     specify one factory to apply the same configuration over all modules, or specify four factories in the order
     *     (FL, FR, BL, BR)
     * @return the current instance of {@link DriveTrainSimulationConfig} for method chaining.
     */
    DriveTrainSimulationConfig& WithSwerveModules(
        std::initializer_list<std::function<SwerveModuleSimulation()>> swerveModuleSimulationFactory);

    /**
     *
     *
     * <h2>Sets the swerve module simulation factory.</h2>
     *
     * <p>Updates the factory used to create swerve module simulations.
     *
     * <p>Uses the same configuration over all the modules
     *
     * @param swerveModuleSimulationFactory the new factory for swerve module simulations.
     * @return the current instance of {@link DriveTrainSimulationConfig} for method chaining.
     */
    DriveTrainSimulationConfig& WithSwerveModule(std::function<SwerveModuleSimulation()> swerveModuleSimulationFactory);

    /**
     *
     *
     * <h2>Sets the gyro simulation factory.</h2>
     *
     * <p>Updates the factory used to create gyro simulations.
     *
     * @param gyroSimulationFactory the new factory for gyro simulations.
     * @return the current instance of {@link DriveTrainSimulationConfig} for method chaining.
     */
    DriveTrainSimulationConfig& WithGyro(std::function<GyroSimulation()> gyroSimulationFactory);

    /**
     *
     *
     * <h2>Calculates the density of the robot.</h2>
     *
     * <p>Returns the density of the robot based on its mass and bumper dimensions.
     *
     * @return the density in kilograms per square meter.
     */
    constexpr decltype(1_kg / 1_sq_m) GetDensity() const { return robotMass / (bumperLengthX * bumperWidthY); }

    /**
     *
     *
     * <h2>Calculates the track length in the X direction.</h2>
     *
     * <p>Returns the total distance between the frontmost and rearmost module translations in the X direction.
     *
     * @return the track length.
     * @throws IllegalStateException if the module translations are empty.
     */
    units::meter_t TrackLengthX() const;

    /**
     *
     *
     * <h2>Calculates the track width in the Y direction.</h2>
     *
     * <p>Returns the total distance between the leftmost and rightmost module translations in the Y direction.
     *
     * @return the track width.
     * @throws IllegalStateException if the module translations are empty.
     */
    units::meter_t TrackWidthY() const;

    inline units::meter_t DriveBaseRadius() const { return units::math::hypot(TrackLengthX(), TrackWidthY()); }

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
