#pragma once
#include "maplesim/simulation/drivesims/configs/SwerveModuleSimulationConfig.h"
#include "maplesim/simulation/drivesims/GyroSimulation.h"

namespace maplesim {

class COTS {
   public:
    /**
     * creates a <a href="https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module">SDS Mark4
     * Swerve Module</a> for simulation
     */
    static SwerveModuleSimulationConfig OfMark4(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel);

    /**
     * creates a <a href="https://www.swervedrivespecialties.com/collections/kits/products/mk4i-swerve-module">SDS
     * Mark4-i Swerve Module</a> for simulation
     */
    static SwerveModuleSimulationConfig OfMark4i(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel);

    /**
     * creates a <a href="https://www.swervedrivespecialties.com/products/mk4n-swerve-module">SDS Mark4-n Swerve
     * Module</a> for simulation
     */
    static SwerveModuleSimulationConfig OfMark4n(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel);

    /**
     * creates a <a href="https://www.swervedrivespecialties.com/products/mk5n-swerve-module">SDS Mark5-n Swerve
     * Module</a> for simulation
     */
    static SwerveModuleSimulationConfig OfMark5n(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel);

    /**
     * creates a <a href="https://www.swervedrivespecialties.com/products/mk5i-swerve-module">SDS Mark5-i Swerve
     * Module</a> for simulation
     */
    static SwerveModuleSimulationConfig OfMark5i(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel);

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x">WCP SwerveX Swerve Module</a>
     * for simulation
     */
    static SwerveModuleSimulationConfig OfSwerveX(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel,
                                                  double firstStageRatio);

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x-flipped">WCP SwerveX Flipped
     * Swerve Module</a> for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6 <br>
     * X3 Ratios are gearRatioLevel 7-9
     */
    static SwerveModuleSimulationConfig OfSwerveXFlipped(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF,
                                                         int gearRatioLevel, int pinionSize);

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-xs">WCP SwerveXS Swerve
     * Module</a> for simulation
     */
    static SwerveModuleSimulationConfig OfSwerveXS(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel,
                                                   int pinionSize);

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x2">WCP SwerveX2 Swerve
     * Module</a> for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6 <br>
     * X3 Ratios are gearRatioLevel 7-9 <br>
     * X4 Ratios are gearRatioLevel 10-12
     */
    static SwerveModuleSimulationConfig OfSwerveX2(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel,
                                                   int pinionSize);

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x2-s">WCP SwerveX2S Swerve
     * Module</a> for simulation
     */
    static SwerveModuleSimulationConfig OfSwerveX2S(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel,
                                                    int pinionSize);

    /**
     * Creates a <a href="https://www.revrobotics.com/rev-21-3005/">REV MAXSwerve swerve module</a> for simulation
     *
     * <p>Base Kit ratios are gearRatioLevel 1-3<br>
     * Gear Ratio Upgrade Kit ratios are gearRatioLevel 4-8
     */
    static SwerveModuleSimulationConfig OfMAXSwerve(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel);

    /**
     * Creates a <a href="https://www.thethriftybot.com/products/thrifty-swerve">TTB Thrifty Swerve swerve module</a>
     * for simulation
     */
    static SwerveModuleSimulationConfig OfThriftySwerve(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF,
                                                        int gearRatioLevel);

    /**
     *
     *
     * <h2>Creates the Simulation for a <a href="https://store.ctr-electronics.com/pigeon-2/">CTRE Pigeon 2 IMU</a>.
     * </h2>
     *
     * @return a gyro simulation factory configured for the Pigeon 2 IMU
     */
    inline static GyroSimulation OfPigeon2() { return {0.5_deg, 0.02}; }

    /**
     *
     *
     * <h2>Creates the Simulation for a <a href="https://pdocs.kauailabs.com/navx-mxp/">navX2-MXP IMU</a>.</h2>
     *
     * @return a gyro simulation factory configured for the navX2-MXP IMU
     */
    inline static GyroSimulation OfNav2X() { return {2_deg, 0.04}; }

    /**
     *
     *
     * <h2>Creates the Simulation for a Generic, Low-Accuracy IMU.</h2>
     *
     * @return a gyro simulation factory configured for a generic low-accuracy IMU
     */
    inline static GyroSimulation OfGenericGyro() { return {5_deg, 0.06}; }
};

}  // namespace maplesim
