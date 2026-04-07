#pragma once
#include "maplesim/simulation/drivesims/configs/SwerveModuleSimulationConfig.h"

namespace maplesim {

class COTS {
   public:
    constexpr static SwerveModuleSimulationConfig OfMark4(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF,
                                                          int gearRatioLevel) {
        double gearing;
        if (gearRatioLevel == 1)
            gearing = 8.14;
        else if (gearRatioLevel == 2)
            gearing = 6.75;
        else if (gearRatioLevel == 3)
            gearing = 6.12;
        else if (gearRatioLevel == 4)
            gearing = 5.14;
        else
            throw std::invalid_argument{fmt::format("Unknown gearing level: {}", gearRatioLevel)};
        return SwerveModuleSimulationConfig{driveMotor, steerMotor, gearing, 12.8, 0.1_V, 0.2_V, 2_in, 0.03_kg_sq_m, wheelCOF};
    }

    constexpr static SwerveModuleSimulationConfig OfMark4i(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF,
                                                           int gearRatioLevel) {
        double gearing;
        if (gearRatioLevel == 1)
            gearing = 8.14;
        else if (gearRatioLevel == 2)
            gearing = 6.75;
        else if (gearRatioLevel == 3)
            gearing = 6.12;
        else if (gearRatioLevel == 4)
            gearing = 5.15;
        else
            throw std::invalid_argument{fmt::format("Unknown gearing level: {}", gearRatioLevel)};
        return SwerveModuleSimulationConfig{driveMotor, steerMotor, gearing, 150.0 / 7.0, 0.1_V, 0.2_V, 2_in, 0.03_kg_sq_m, wheelCOF};
    }

    constexpr static SwerveModuleSimulationConfig OfMark4n(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF,
                                                           int gearRatioLevel) {
        double gearing;
        if (gearRatioLevel == 1)
            gearing = 7.13;
        else if (gearRatioLevel == 2)
            gearing = 5.9;
        else if (gearRatioLevel == 3)
            gearing = 5.36;
        else
            throw std::invalid_argument{fmt::format("Unknown gearing level: {}", gearRatioLevel)};
        return SwerveModuleSimulationConfig{driveMotor, steerMotor, gearing, 18.75, 0.1_V, 0.2_V, 2_in, 0.03_kg_sq_m, wheelCOF};
    }

    constexpr static SwerveModuleSimulationConfig OfMark5n(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF,
                                                           int gearRatioLevel) {
        double gearing;
        if (gearRatioLevel == 1)
            gearing = 7.03;
        else if (gearRatioLevel == 2)
            gearing = 6.03;
        else if (gearRatioLevel == 3)
            gearing = 5.27;
        else
            throw std::invalid_argument{fmt::format("Unknown gearing level: {}", gearRatioLevel)};
        return SwerveModuleSimulationConfig{driveMotor, steerMotor, gearing, 287.0 / 11.0, 0.1_V, 0.2_V, 2_in, 0.03_kg_sq_m, wheelCOF};
    }

    constexpr static SwerveModuleSimulationConfig OfMark5i(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF,
                                                           int gearRatioLevel) {
        double gearing;
        if (gearRatioLevel == 1)
            gearing = 7.03;
        else if (gearRatioLevel == 2)
            gearing = 6.03;
        else if (gearRatioLevel == 3)
            gearing = 5.27;
        else
            throw std::invalid_argument{fmt::format("Unknown gearing level: {}", gearRatioLevel)};
        return SwerveModuleSimulationConfig{driveMotor, steerMotor, gearing, 26.0, 0.1_V, 0.2_V, 2_in, 0.03_kg_sq_m, wheelCOF};
    }

    constexpr static SwerveModuleSimulationConfig OfSwerveX(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF,
                                                            int gearRatioLevel, double firstStageRatio) {
        double secondStageRatio;
        if (gearRatioLevel == 1)
            secondStageRatio = 26.0 / 20.0;
        else if (gearRatioLevel == 2)
            secondStageRatio = 28.0 / 18.0;
        else if (gearRatioLevel == 3)
            secondStageRatio = 28.0 / 18.0;
        else
            throw std::invalid_argument{fmt::format("Unknown gearing level: {}", gearRatioLevel)};
        return SwerveModuleSimulationConfig{driveMotor,   steerMotor, firstStageRatio * secondStageRatio, 11.3142, 0.1_V, 0.2_V, 2_in,
                                            0.03_kg_sq_m, wheelCOF};
    }

    constexpr static SwerveModuleSimulationConfig OfSwerveXFlipped(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF,
                                                                   int gearRatioLevel, int pinionSize) {
        double gearing;
        if (gearRatioLevel == 1) {
            if (pinionSize == 10)
                gearing = 8.1;
            else if (pinionSize == 11)
                gearing = 7.36;
            else if (pinionSize == 12)
                gearing = 6.75;
            else
                throw std::invalid_argument{fmt::format("Unknown pinion size: {}", pinionSize)};
        } else if (gearRatioLevel == 2) {
            if (pinionSize == 10)
                gearing = 6.72;
            else if (pinionSize == 11)
                gearing = 6.11;
            else if (pinionSize == 12)
                gearing = 5.6;
            else
                throw std::invalid_argument{fmt::format("Unknown pinion size: {}", pinionSize)};
        } else if (gearRatioLevel == 3) {
            if (pinionSize == 10)
                gearing = 5.51;
            else if (pinionSize == 11)
                gearing = 5.01;
            else if (pinionSize == 12)
                gearing = 4.59;
            else
                throw std::invalid_argument{fmt::format("Unknown pinion size: {}", pinionSize)};
        } else
            throw std::invalid_argument{fmt::format("Unknown gearing level: {}", gearRatioLevel)};
        return SwerveModuleSimulationConfig{driveMotor, steerMotor, gearing, 13.3714, 0.1_V, 0.2_V, 2_in, 0.03_kg_sq_m, wheelCOF};
    }

    constexpr static SwerveModuleSimulationConfig OfSwerveXS(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF,
                                                             int gearRatioLevel, int pinionSize) {
        double gearing;
        if (gearRatioLevel == 1) {
            if (pinionSize == 12)
                gearing = 6;
            else if (pinionSize == 13)
                gearing = 5.54;
            else if (pinionSize == 14)
                gearing = 5.14;
            else
                throw std::invalid_argument{fmt::format("Unknown pinion size: {}", pinionSize)};
        } else if (gearRatioLevel == 2) {
            if (pinionSize == 12)
                gearing = 4.71;
            else if (pinionSize == 13)
                gearing = 4.4;
            else if (pinionSize == 14)
                gearing = 4.13;
            else
                throw std::invalid_argument{fmt::format("Unknown pinion size: {}", pinionSize)};
        } else
            throw std::invalid_argument{fmt::format("Unknown gearing level: {}", gearRatioLevel)};
        return SwerveModuleSimulationConfig{driveMotor, steerMotor, gearing, 41.25, 0.1_V, 0.2_V, 2_in, 0.03_kg_sq_m, wheelCOF};
    }

    constexpr static SwerveModuleSimulationConfig OfSwerveX2(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF,
                                                             int gearRatioLevel, int pinionSize) {
        double gearing;
        if (gearRatioLevel == 1) {
            if (pinionSize == 10)
                gearing = 7.67;
            else if (pinionSize == 11)
                gearing = 6.98;
            else if (pinionSize == 12)
                gearing = 6.39;
            else
                throw std::invalid_argument{fmt::format("Unknown pinion size: {}", pinionSize)};
        } else if (gearRatioLevel == 2) {
            if (pinionSize == 10)
                gearing = 6.82;
            else if (pinionSize == 11)
                gearing = 6.2;
            else if (pinionSize == 12)
                gearing = 5.68;
            else
                throw std::invalid_argument{fmt::format("Unknown pinion size: {}", pinionSize)};
        } else if (gearRatioLevel == 3) {
            if (pinionSize == 10)
                gearing = 6.48;
            else if (pinionSize == 11)
                gearing = 5.89;
            else if (pinionSize == 12)
                gearing = 5.4;
            else
                throw std::invalid_argument{fmt::format("Unknown pinion size: {}", pinionSize)};
        } else if (gearRatioLevel == 4) {
            if (pinionSize == 10)
                gearing = 5.67;
            else if (pinionSize == 11)
                gearing = 5.15;
            else if (pinionSize == 12)
                gearing = 4.73;
            else
                throw std::invalid_argument{fmt::format("Unknown pinion size: {}", pinionSize)};
        } else
            throw std::invalid_argument{fmt::format("Unknown gearing level: {}", gearRatioLevel)};
        return SwerveModuleSimulationConfig{driveMotor, steerMotor, gearing, 12.1, 0.1_V, 0.2_V, 2_in, 0.03_kg_sq_m, wheelCOF};
    }

    constexpr static SwerveModuleSimulationConfig OfSwerveX2S(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF,
                                                              int gearRatioLevel, int pinionSize) {
        double gearing;
        if (gearRatioLevel == 1) {
            if (pinionSize == 15)
                gearing = 6;
            else if (pinionSize == 16)
                gearing = 5.63;
            else if (pinionSize == 17)
                gearing = 5.29;
            else
                throw std::invalid_argument{fmt::format("Unknown pinion size: {}", pinionSize)};
        } else if (gearRatioLevel == 2) {
            if (pinionSize == 17)
                gearing = 4.94;
            else if (pinionSize == 18)
                gearing = 4.67;
            else if (pinionSize == 19)
                gearing = 4.42;
            else
                throw std::invalid_argument{fmt::format("Unknown pinion size: {}", pinionSize)};
        } else if (gearRatioLevel == 3) {
            if (pinionSize == 19)
                gearing = 4.11;
            else if (pinionSize == 20)
                gearing = 3.9;
            else if (pinionSize == 21)
                gearing = 3.71;
            else
                throw std::invalid_argument{fmt::format("Unknown pinion size: {}", pinionSize)};
        } else if (gearRatioLevel == 4) {
            if (pinionSize == 10)
                gearing = 5.67;
            else if (pinionSize == 11)
                gearing = 5.15;
            else if (pinionSize == 12)
                gearing = 4.73;
            else
                throw std::invalid_argument{fmt::format("Unknown pinion size: {}", pinionSize)};
        } else
            throw std::invalid_argument{fmt::format("Unknown gearing level: {}", gearRatioLevel)};
        return SwerveModuleSimulationConfig{driveMotor, steerMotor, gearing, 25.9, 0.1_V, 0.2_V, 2_in, 0.03_kg_sq_m, wheelCOF};
    }

    constexpr static SwerveModuleSimulationConfig OfMAXSwerve(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF,
                                                              int gearRatioLevel) {
        double gearing;
        if (gearRatioLevel == 1)
            gearing = 5.5;
        else if (gearRatioLevel == 2)
            gearing = 5.08;
        else if (gearRatioLevel == 3)
            gearing = 4.71;
        else if (gearRatioLevel == 4)
            gearing = 4.5;
        else if (gearRatioLevel == 5)
            gearing = 4.29;
        else if (gearRatioLevel == 6)
            gearing = 4;
        else if (gearRatioLevel == 7)
            gearing = 3.75;
        else if (gearRatioLevel == 8)
            gearing = 3.56;
        else
            throw std::invalid_argument{fmt::format("Unknown gearing level: {}", gearRatioLevel)};
        return SwerveModuleSimulationConfig{driveMotor, steerMotor, gearing, 9424.0 / 203.0, 0.1_V, 0.1_V, 1.5_in, 0.02_kg_sq_m, wheelCOF};
    }

    constexpr static SwerveModuleSimulationConfig OfThriftySwerve(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF,
                                                                  int gearRatioLevel) {
        double gearing;
        if (gearRatioLevel == 1)
            gearing = 6.75;
        else if (gearRatioLevel == 2)
            gearing = 6.23;
        else if (gearRatioLevel == 3)
            gearing = 5.79;
        else if (gearRatioLevel == 4)
            gearing = 6;
        else if (gearRatioLevel == 5)
            gearing = 5.54;
        else if (gearRatioLevel == 6)
            gearing = 5.14;
        else
            throw std::invalid_argument{fmt::format("Unknown gearing level: {}", gearRatioLevel)};
        return SwerveModuleSimulationConfig{driveMotor, steerMotor, gearing, 25, 0.1_V, 0.1_V, 1.5_in, 0.02_kg_sq_m, wheelCOF};
    }
};

}  // namespace maplesim
