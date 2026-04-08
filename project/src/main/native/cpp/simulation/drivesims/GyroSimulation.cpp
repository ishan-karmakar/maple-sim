#include <random>
#include <units/angular_acceleration.h>
#include <units/math.h>
#include "maplesim/simulation/drivesims/GyroSimulation.h"
#include "maplesim/simulation/SimulatedArena.h"

using namespace maplesim;

std::random_device GyroSimulation::rd;
std::mt19937 GyroSimulation::gen{rd()};

GyroSimulation::GyroSimulation(units::degree_t AVERAGE_DRIFTING_IN_30_SECS_MOTIONLESS,
                               double VELOCITY_MEASUREMENT_STANDARD_DEVIATION_PERCENT)
      : AVERAGE_DRIFTING_IN_30_SECS_MOTIONLESS{AVERAGE_DRIFTING_IN_30_SECS_MOTIONLESS},
        VELOCITY_MEASUREMENT_STANDARD_DEVIATION_PERCENT{VELOCITY_MEASUREMENT_STANDARD_DEVIATION_PERCENT} {
    for (int i = 0; i < SimulatedArena::GetSimulationSubTicksIn1Period(); i++)
        cachedRotations.push_back(gyroReading);
}

void GyroSimulation::UpdateSimulationSubTick(units::radians_per_second_t actualAngularVelocity) {
    frc::Rotation2d driftingDueToImpact = GetDriftingDueToImpact(actualAngularVelocity);
    gyroReading = gyroReading + driftingDueToImpact;

    frc::Rotation2d dTheta = GetGyroDTheta(actualAngularVelocity);
    gyroReading = gyroReading + dTheta;

    frc::Rotation2d noMotionDrifting = GetNoMotionDrifting();
    gyroReading = gyroReading + noMotionDrifting;

    cachedRotations.push_back(gyroReading);
}

frc::Rotation2d GyroSimulation::GetDriftingDueToImpact(units::radians_per_second_t actualAngularVelocity) {
    units::radians_per_second_squared_t angularAcceleration =
        (actualAngularVelocity - previousAngularVelocity) / SimulatedArena::GetSimulationDt();
    units::radian_t driftingDueToImpactAbs =
        units::math::abs(angularAcceleration) > ANGULAR_ACCELERATION_THRESHOLD_START_DRIFTING
            ? units::math::abs(angularAcceleration) / ANGULAR_ACCELERATION_THRESHOLD_START_DRIFTING * DRIFT_DUE_TO_IMPACT_COEFFICIENT
            : 0_rad;
    units::radian_t driftingDueToImpact = units::math::copysign(driftingDueToImpactAbs, -angularAcceleration);
    previousAngularVelocity = actualAngularVelocity;

    return driftingDueToImpact;
}

frc::Rotation2d GyroSimulation::GetGyroDTheta(units::radians_per_second_t actualAngularVelocity) {
    measuredAngularVelocity = units::radians_per_second_t{std::normal_distribution<double>(
        actualAngularVelocity.value(), VELOCITY_MEASUREMENT_STANDARD_DEVIATION_PERCENT * std::abs(actualAngularVelocity.value()))(gen)};
    return measuredAngularVelocity * SimulatedArena::GetSimulationDt();
}

frc::Rotation2d GyroSimulation::GetNoMotionDrifting() const {
    units::degree_t AVERAGE_DRIFTING_1_PERIOD = AVERAGE_DRIFTING_IN_30_SECS_MOTIONLESS / 30_s * SimulatedArena::GetSimulationDt();
    units::radian_t driftingInThisPeriod = units::degree_t{std::normal_distribution<double>(0, AVERAGE_DRIFTING_1_PERIOD.value())(gen)};
    return driftingInThisPeriod;
}
