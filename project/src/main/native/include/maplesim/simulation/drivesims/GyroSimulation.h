#pragma once
#include <deque>
#include <random>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <frc/geometry/Rotation2d.h>

namespace maplesim {

class GyroSimulation {
    public:
    GyroSimulation(units::degree_t AVERAGE_DRIFTING_IN_30_SECS_MOTIONLESS, double VELOCITY_MEASUREMENT_STANDARD_DEVIATION_PERCENT);

    constexpr void SetRotation(frc::Rotation2d currentRotation) {
        gyroReading = currentRotation;
    }

    constexpr frc::Rotation2d GetGyroReading() const { return gyroReading; }

    constexpr units::radians_per_second_t GetMeasuredAngularVelocity() const {
        return measuredAngularVelocity;
    }

    std::vector<frc::Rotation2d> GetCachedGyroReadings() const;

    void UpdateSimulationSubTick(units::radians_per_second_t actualAngularVelocity);

    private:
    frc::Rotation2d GetDriftingDueToImpact(units::radians_per_second_t actualAngularVelocity);
    frc::Rotation2d GetGyroDTheta(units::radians_per_second_t actualAngularVelocity);
    frc::Rotation2d GetNoMotionDrifting() const;

    static constexpr units::radians_per_second_squared_t ANGULAR_ACCELERATION_THRESHOLD_START_DRIFTING = 500_rad_per_s_sq;
    static constexpr units::radian_t DRIFT_DUE_TO_IMPACT_COEFFICIENT = 1_deg;

    units::degree_t AVERAGE_DRIFTING_IN_30_SECS_MOTIONLESS;
    double VELOCITY_MEASUREMENT_STANDARD_DEVIATION_PERCENT;

    frc::Rotation2d gyroReading;
    units::radians_per_second_t measuredAngularVelocity;
    units::radians_per_second_t previousAngularVelocity;
    std::deque<frc::Rotation2d> cachedRotations;

    static std::random_device rd;
    static std::mt19937 gen;
};

}