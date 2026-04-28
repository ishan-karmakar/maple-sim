#pragma once
#include <deque>
#include <random>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <frc/geometry/Rotation2d.h>

namespace maplesim {

/**
 * Simulation for a IMU module used as gyro.
 *
 * <p>The Simulation is basically an indefinite integral of the angular velocity during each simulation sub ticks. Above
 * that, it also musicales the measurement inaccuracy of the gyro, drifting in no-motion and drifting due to impacts.
 */
class GyroSimulation {
   public:
    /**
     *
     *
     * <h2>Creates a Gyro Simulation.</h2>
     *
     * @param AVERAGE_DRIFTING_IN_30_SECS_MOTIONLESS_DEG the average amount of drift, in degrees, the gyro experiences
     *     if it remains motionless for 30 seconds on a vibrating platform. This value can often be found in the user
     *     manual.
     * @param VELOCITY_MEASUREMENT_STANDARD_DEVIATION_PERCENT the standard deviation of the velocity measurement,
     *     typically around 0.05
     */
    GyroSimulation(units::degree_t AVERAGE_DRIFTING_IN_30_SECS_MOTIONLESS, double VELOCITY_MEASUREMENT_STANDARD_DEVIATION_PERCENT);

    /**
     *
     *
     * <h2>Calibrates the Gyro to a Given Rotation.</h2>
     *
     * <p>This method sets the current rotation of the gyro, similar to <code>Pigeon2().setYaw()
     * </code>.
     *
     * <p>After setting the rotation, the gyro will continue to estimate the rotation by integrating the angular
     * velocity, adding it to the specified rotation.
     *
     * @param currentRotation the current rotation of the robot, represented as a {@link Rotation2d}
     */
    constexpr void SetRotation(frc::Rotation2d currentRotation) { gyroReading = currentRotation; }

    /**
     *
     *
     * <h2>Obtains the Estimated Rotation of the Gyro.</h2>
     *
     * <p>This method returns the estimated rotation of the gyro, which includes measurement errors due to drifting and
     * other factors.
     *
     * @return the current reading of the gyro, represented as a {@link Rotation2d}
     */
    constexpr frc::Rotation2d GetGyroReading() const { return gyroReading; }

    /**
     *
     *
     * <h2>Gets the Measured Angular Velocity of the Gyro.</h2>
     *
     * <p>This method returns the angular velocity measured by the gyro, in radians per second.
     *
     * <p>The measurement includes random errors based on the configured settings of the gyro.
     *
     * @return the measured angular velocity
     */
    constexpr units::radians_per_second_t GetMeasuredAngularVelocity() const { return measuredAngularVelocity; }

    /**
     * gyro readings for <a
     * href="https://v6.docs.ctr-electronics.com/en/stable/docs/application-notes/update-frequency-impact.html">high-frequency
     * odometers</a>.
     *
     * @return the readings of the gyro during the last 5 simulation sub ticks
     */
    std::vector<frc::Rotation2d> GetCachedGyroReadings() const;

    /**
     *
     *
     * <h2>Updates the Gyro Simulation for Each Sub-Tick.</h2>
     *
     * <p>This method updates the gyro simulation and should be called during every sub-tick of the simulation.
     *
     * <p>If you are using this class outside of {@link org.ironmaple.simulation.SimulatedArena}: make sure to call it 5
     * times in each robot period (if using default timings), or refer to
     * {@link org.ironmaple.simulation.SimulatedArena#overrideSimulationTimings(Time, int)}.
     *
     * @param actualAngularVelocityRadPerSec the actual angular velocity in radians per second, usually obtained from
     *     {@link AbstractDriveTrainSimulation#getAngularVelocity()}
     */
    void UpdateSimulationSubTick(units::radians_per_second_t actualAngularVelocity);

   private:
    /**
     *
     *
     * <h2>Simulates IMU Drifting Due to Robot Impacts.</h2>
     *
     * <p>This method generates a random amount of drifting for the IMU if the instantaneous angular acceleration
     * exceeds a threshold, simulating the effects of impacts on the robot.
     *
     * @param actualAngularVelocityRadPerSec the actual angular velocity in radians per second, used to determine if an
     *     impact is detected
     * @return the amount of drifting the IMU will experience if an impact is detected, or <code>
     *     Rotation2d.fromRadians(0)</code> if no impact is detected
     */
    frc::Rotation2d GetDriftingDueToImpact(units::radians_per_second_t actualAngularVelocity);

    /**
     *
     *
     * <h2>Gets the Measured ΔTheta of the Gyro.</h2>
     *
     * <p>This method simulates the change in the robot's angle (ΔTheta) since the last sub-tick, as measured by the
     * gyro.
     *
     * <p>The measurement includes random errors based on the configuration of the gyro.
     *
     * @param actualAngularVelocityRadPerSec the actual angular velocity in radians per second, used to calculate the
     *     ΔTheta
     * @return the measured ΔTheta, including any measurement errors
     */
    frc::Rotation2d GetGyroDTheta(units::radians_per_second_t actualAngularVelocity);

    /**
     *
     *
     * <h2>Generates the No-Motion Gyro Drifting.</h2>
     *
     * <p>This method simulates the minor drifting of the gyro that occurs regardless of whether the robot is moving or
     * not.
     *
     * @return the amount of drifting generated while the robot is not moving
     */
    frc::Rotation2d GetNoMotionDrifting() const;

    /* The threshold of instantaneous angular acceleration at which the chassis is considered to experience an "impact." */
    static constexpr units::radians_per_second_squared_t ANGULAR_ACCELERATION_THRESHOLD_START_DRIFTING = 500_rad_per_s_sq;
    /* The amount of drift, in radians, that the gyro experiences as a result of each multiple of the angular acceleration threshold. */
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

}  // namespace maplesim
