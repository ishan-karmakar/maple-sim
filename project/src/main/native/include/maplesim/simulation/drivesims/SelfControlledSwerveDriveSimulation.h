#pragma once
#include <frc/controller/PIDController.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include "maplesim/simulation/drivesims/SwerveDriveSimulation.h"

namespace maplesim {

class SelfControlledSwerveDriveSimulation {
   public:
    class SelfControlledModuleSimulation {
       public:
        SelfControlledModuleSimulation(SwerveModuleSimulation moduleSimulation);

        constexpr SelfControlledModuleSimulation& WithSteerPID(frc::PIDController steerController) {
            this->steerController = steerController;
            this->steerController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
            return *this;
        }

        constexpr SelfControlledModuleSimulation& WithCurrentLimits(units::ampere_t driveCurrentLimit, units::ampere_t steerCurrentLimit) {
            this->driveCurrentLimit = driveCurrentLimit;
            driveMotor.WithCurrentLimit(driveCurrentLimit);
            steerMotor.WithCurrentLimit(steerCurrentLimit);
            steerController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
            return *this;
        }

        /**
         *
         *
         * <h2>Runs the control loops for swerve states on a simulated module.</h2>
         *
         * <p>Optimizes the set-point using {@link SwerveModuleState#optimize(SwerveModuleState, Rotation2d)}.
         *
         * <p>Executes a closed-loop control on the swerve module.
         *
         * @param setPoint the desired state to optimize and apply
         * @return the optimized swerve module state after control execution
         */
        constexpr frc::SwerveModuleState OptimizeAndRunModuleState(frc::SwerveModuleState setPoint) {
            setPoint.Optimize(instance.GetSteerAbsoluteFacing());
            RunModuleState(setPoint);
            return setPoint;
        }

        constexpr void RunModuleState(frc::SwerveModuleState setPoint) {
            units::meters_per_second_t cosProjectedSpeed = setPoint.speed * (setPoint.angle - instance.GetSteerAbsoluteFacing()).Cos();
            units::radians_per_second_t driveWheelVelocitySetPoint = cosProjectedSpeed / instance.config.WHEEL_RADIUS * 1_rad;

            driveMotor.RequestVoltage(instance.config.driveMotorConfigs.CalculateVoltage(0_A, driveWheelVelocitySetPoint));

            steerMotor.RequestVoltage(units::volt_t{
                steerController.Calculate(instance.GetSteerAbsoluteFacing().Radians().value(), setPoint.angle.Radians().value())});
        }

        constexpr void RunDriveMotorCharacterization(frc::Rotation2d desiredModuleFacing, units::volt_t volts) {
            driveMotor.RequestVoltage(volts);

            steerMotor.RequestVoltage(units::volt_t{
                steerController.Calculate(instance.GetSteerAbsoluteFacing().Radians().value(), desiredModuleFacing.Radians().value())});
        }

        constexpr void RunSteerMotorCharacterization(units::volt_t volts) {
            driveMotor.RequestVoltage(0_V);
            steerMotor.RequestVoltage(volts);
        }

        constexpr frc::SwerveModuleState GetMeasuredState() const { return instance.GetCurrentState(); }

        constexpr frc::SwerveModulePosition GetModulePosition() const {
            return {instance.GetDriveWheelFinalPosition() * instance.config.WHEEL_RADIUS / 1_rad, instance.GetSteerAbsoluteFacing()};
        }

        SwerveModuleSimulation instance;

       private:
        units::ampere_t driveCurrentLimit;

        frc::PIDController steerController;

        GenericMotorController driveMotor;
        GenericMotorController steerMotor;
    };

    /**
     *
     *
     * <h2>Default Constructor.</h2>
     *
     * <p>Constructs a simplified swerve simulation with default standard deviations for odometry & vision pose
     * estimates.
     *
     * <p>Odometry is simulated as high-frequency odometry, assuming a robot period of 0.02 seconds and an odometry
     * frequency of 250 Hz.
     *
     * @param swerveDriveSimulation the {@link SwerveDriveSimulation} to control.
     */
    inline SelfControlledSwerveDriveSimulation(const SwerveDriveSimulation& swerveDriveSimulation)
          : SelfControlledSwerveDriveSimulation{swerveDriveSimulation, {0.1, 0.1, 0.1}, {0.9, 0.9, 0.9}} {}

    /**
     *
     *
     * <h2>Constructs an instance with given odometry standard deviations.</h2>
     *
     * <p>Constructs a simplified swerve simulation with specified standard deviations for odometry & vision pose
     * estimates.
     *
     * @param swerveDriveSimulation the {@link SwerveDriveSimulation} to control.
     * @param stateStdDevs the standard deviations for odometry encoders.
     * @param visionMeasurementStdDevs the standard deviations for vision pose estimates.
     */
    SelfControlledSwerveDriveSimulation(const SwerveDriveSimulation& swerveDriveSimulation, const wpi::array<double, 3>& stateStdDevs,
                                        const wpi::array<double, 3>& visionMeasurementStdDevs);

    /**
     *
     *
     * <h2>Periodic Method for Simplified Swerve Sim.</h2>
     *
     * <p>Call this method in the {@link edu.wpi.first.wpilibj2.command.Subsystem#periodic()} of your swerve subsystem.
     *
     * <p>Updates the odometry by fetching cached inputs.
     */
    void Periodic();

    /**
     *
     *
     * <h2>Obtains the LATEST module positions measured by the encoders.</h2>
     *
     * <p>The order for swerve modules is: front-left, front-right, back-left, back-right.
     *
     * @return the module positions
     */
    constexpr std::vector<frc::SwerveModulePosition> GetLatestModulePositions() const {
        std::vector<frc::SwerveModulePosition> positions;
        for (const auto& sim : moduleSimulations)
            positions.push_back(sim.GetModulePosition());
        return positions;
    }

    /**
     *
     *
     * <h2>Obtains the CACHED module positions measured by the encoders.</h2>
     *
     * <p>This simulates high-frequency odometry.
     *
     * <p>The module positions, or the value of {@link #getLatestModulePositions()} are cached in every simulation
     * sub-tick.
     *
     * <p>The array is ordered in a [timeStampIndex][moduleIndex] format.
     *
     * <p>The order for swerve modules is: front-left, front-right, back-left, back-right.
     *
     * @return the cached module positions
     */
    std::vector<std::vector<frc::SwerveModulePosition>> GetCachedModulePositions() const;

    /**
     *
     *
     * <h2>Obtains the robot pose measured by the odometry.</h2>
     *
     * <p>Retrieves the pose estimated by the odometry system (and vision, if applicable).
     *
     * <p>This method wraps around {@link SwerveDrivePoseEstimator#getEstimatedPosition()}.
     *
     * <p>This represents the estimated position of the robot.
     *
     * <p>Note that this estimation includes realistic simulations of measurement errors due to skidding and odometry
     * drift.
     *
     * <p>To obtain the ACTUAL pose of the robot, with no measurement errors, use
     * {@link #getActualPoseInSimulationWorld()}.
     */
    constexpr frc::Pose2d GetOdometryEstimatedPose() const {
        return poseEstimator.GetEstimatedPosition();
    }

    /**
     *
     *
     * <h2>Resets the odometry to a specified position.</h2>
     *
     * <p>This method wraps around {@link SwerveDrivePoseEstimator#resetPosition(Rotation2d, SwerveModulePosition[],
     * Pose2d)}.
     *
     * <p>It resets the position of the pose estimator to the given pose.
     *
     * @param pose The position on the field where the robot is located.
     */
    void ResetOdometry(frc::Pose2d pose);

   private:
    SwerveDriveSimulation swerveDriveSimulation;
    std::vector<SelfControlledModuleSimulation> moduleSimulations;
    frc::SwerveDriveKinematics<4> kinematics;
    frc::SwerveDrivePoseEstimator<4> poseEstimator;
    std::vector<frc::SwerveModuleState> setPointsOptimized;
};

}  // namespace maplesim
