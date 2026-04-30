#pragma once
#include <unordered_set>
#include <units/acceleration.h>
#include <units/velocity.h>
#include <frc/Timer.h>
#include <Eigen/Geometry>
#include "maplesim/simulation/gamepieces/GamePieceOnFieldSimulation.h"

namespace maplesim {

class SimulatedArena;

/**
 *
 *
 * <h1>Simulates a Game Piece Launched into the Air</h1>
 *
 * <p>Check<a href='https://shenzhen-robotics-alliance.github.io/maple-sim/simulating-projectiles/'>Online
 * Documentation</a>
 *
 * <p>The movement is modeled by simple projectile motion.
 *
 * <p>If the projectile flies off the field, touches the ground, or hits its target, it will be automatically removed.
 *
 * <h3>Additional Features:</h3>
 *
 * <ul>
 *   <li>Optionally, it can be configured to become a {@link GamePieceOnFieldSimulation} upon touching the ground.
 *   <li>Optionally, it can be configured to have a "desired target." Upon hitting the target, it can be configured to
 *       run a callback.
 * </ul>
 *
 * <h3>Limitations:</h3>
 *
 * <ul>
 *   <li>Air drag is ignored.
 *   <li><strong>DOES NOT</strong> have collision space when flying.
 * </ul>
 */
class GamePieceProjectile : GamePiece {
   public:
    /**
     *
     *
     * <h2>Creates a Game Piece Projectile Ejected from a Shooter.</h2>
     *
     * @param info the info of the game piece
     * @param robotPosition the position of the robot (not the shooter) at the time of launching the game piece
     * @param shooterPositionOnRobot the translation from the shooter's position to the robot's center, in the robot's
     *     frame of reference
     * @param chassisSpeedsFieldRelative the field-relative velocity of the robot chassis when launching the game piece,
     *     influencing the initial velocity of the game piece
     * @param shooterFacing the direction in which the shooter is facing at launch
     * @param initialHeight the initial height of the game piece when launched, i.e., the height of the shooter from the
     *     ground
     * @param launchingSpeed the speed at which the game piece is launch
     * @param shooterAngle the pitch angle of the shooter when launching
     */
    inline GamePieceProjectile(GamePieceOnFieldSimulation::GamePieceInfo info, frc::Translation2d robotPosition,
                               frc::Translation2d shooterPositionOnRobot, frc::ChassisSpeeds chassisSpeedsFieldRelative,
                               frc::Rotation2d shooterFacing, units::meter_t initialHeight, units::meters_per_second_t launchingSpeed,
                               units::radian_t shooterAngle)
          : GamePieceProjectile{info,
                                robotPosition + shooterPositionOnRobot.RotateBy(shooterFacing),
                                CalculateInitialProjectileVelocity(shooterPositionOnRobot, chassisSpeedsFieldRelative, shooterFacing,
                                                                   launchingSpeed * units::math::cos(shooterAngle)),
                                initialHeight,
                                launchingSpeed * units::math::sin(shooterAngle),
                                {0_rad, -shooterAngle, shooterFacing.Radians()}} {}

    /**
     *
     *
     * <h2>Creates a Game Piece Projectile Ejected from a Shooter.</h2>
     *
     * @param info the info of the game piece
     * @param initialPosition the position of the game piece at the moment it is launched into the air
     * @param initialLaunchingVelocityMPS the horizontal component of the initial velocity in the X-Y plane, in meters
     *     per second (m/s)
     * @param initialHeight the initial height of the game piece when launched (the height of the shooter from the
     *     ground)
     * @param initialVerticalSpeedMPS the vertical component of the initial velocity, in meters per second (m/s)
     * @param gamePieceRotation the 3D rotation of the game piece during flight (only affects visualization of the game
     *     piece)
     */
    inline GamePieceProjectile(GamePieceOnFieldSimulation::GamePieceInfo info, frc::Translation2d initialPosition,
                               Eigen::Vector2d intiialLaunchingVelocityMPS, units::meter_t initialHeight,
                               units::meters_per_second_t initialVerticalSpeed, frc::Rotation3d gamePieceRotation)
          : info{info},
            gamePieceType{info.type},
            initialPosition{initialPosition},
            initialLaunchingVelocity{initialLaunchingVelocity},
            initialHeight{initialHeight},
            initialVerticalSpeed{initialVerticalSpeed},
            gamePieceRotation{gamePieceRotation} {}

    /**
     *
     *
     * <h2>Starts the Game Piece Projectile Simulation.</h2>
     *
     * <p>This method initiates the projectile motion of the game piece with the following actions:
     *
     * <ul>
     *   <li>Initiates the projectile motion of the game piece. The current pose can be obtained with
     *       {@link #getPose3d()}.
     *   <li>Calculates whether the projectile will hit the target during its flight. The result can be obtained using
     *       {@link #willHitTarget()}.
     *   <li>Calculates a preview trajectory by simulating the projectile's motion for up to 100 steps, with each step
     *       lasting 0.02 seconds.
     *   <li>If specified, displays the trajectory using
     *       {@link GamePieceProjectile#projectileTrajectoryDisplayCallBackHitTarget}, which can be set via
     *       {@link GamePieceProjectile#withProjectileTrajectoryDisplayCallBack(Consumer)}.
     *   <li>Starts the {@link #launchedTimer}, which stores the amount of time elapsed after the game piece is launched
     * </ul>
     */
    void Launch();

    /**
     *
     *
     * <h2>Checks if the Game Piece Has Touched the Ground.</h2>
     *
     * <p>This method determines whether the game piece has touched the ground at the current time.
     *
     * <ul>
     *   <li>The result is calculated during the {@link #launch()} method.
     *   <li>Before calling {@link #launch()}, this method will always return <code>false</code>.
     * </ul>
     *
     * @return <code>true</code> if the game piece has touched the ground, otherwise <code>false
     *     </code>
     */
    inline bool HasHitGround() const {
        return GetPositionAtTime(launchedTimer.Get()).Z() <= heightAsTouchGround && launchedTimer.Get() * GRAVITY > initialVerticalSpeed;
    }

    /**
     *
     *
     * <h2>Checks if the Game Piece Has Flown Out of the Field's Boundaries.</h2>
     *
     * <p>This method determines whether the game piece has flown out of the field's boundaries (outside the fence).
     *
     * <ul>
     *   <li>The result is calculated during the {@link #launch()} method.
     *   <li>Before calling {@link #launch()}, this method will always return <code>false</code>.
     * </ul>
     *
     * @return <code>true</code> if the game piece has flown out of the field's boundaries, otherwise <code>false</code>
     */
    inline bool HasGoneOutOfField() const { return IsOutOfField(launchedTimer.Get()); }

    /**
     *
     *
     * <h2>Checks if the projectile will hit the target <strong>AT SOME MOMENT</strong> during its flight</h2>
     *
     * <ul>
     *   <li>The result is calculated during the {@link #launch()} method.
     *   <li>Before calling {@link #launch()}, this method will always return <code>false</code>.
     * </ul>
     *
     * <p>This is different from {@link #hasHitTarget()}
     */
    constexpr bool WillHitTarget() const { return calculatedHitTargetTime != 1_s; }

    /**
     *
     *
     * <h2>Checks if the Projectile Has Already Hit the Target <strong>At the Moment</strong>.</h2>
     *
     * <p>This method checks whether the projectile has hit the target at the current time.
     *
     * <ul>
     *   <li>Before calling {@link #launch()}, this method will always return <code>false</code>.
     *   <li>This is different from {@link #willHitTarget()}, which predicts whether the projectile will eventually hit
     *       the target.
     * </ul>
     *
     * @return <code>true</code> if the projectile has hit the target at the current time, otherwise <code>false</code>
     */
    inline bool HasHitTarget() const { return WillHitTarget() && launchedTimer.Get() >= calculatedHitTargetTime; }

    /**
     *
     *
     * <h2>Clean up the trajectory through {@link #projectileTrajectoryDisplayCallBackHitTarget}</h2>
     *
     * @return this instance
     */
    inline GamePieceProjectile& CleanUp() {
        projectileTrajectoryDisplayCallBackHitTarget({});
        projectileTrajectoryDisplayCallBackMiss({});
        return *this;
    }

    /**
     *
     *
     * <h2>Calculates the Projectile's Current Position.</h2>
     *
     * <p>The position is calculated using {@link #getPositionAtTime(double)} while the rotation is pre-stored.
     *
     * @return a {@link Pose3d} object representing the current pose of the game piece
     */
    inline frc::Pose3d GetPose3d() const override { return {GetPositionAtTime(launchedTimer.Get()), gamePieceRotation}; }

    /**
     *
     *
     * <h2>Calculates the Projectile's Velocity at a Given Time.</h2>
     *
     * @see #getVelocityMPSAtTime(double)
     * @return a {@link Translation3d} object representing the calculated 3d velocity of the projectile at time <code>t
     *     </code>, in meters per second
     */
    inline Eigen::Vector3d GetVelocity3dMPS() const override { return GetVelocityAtTime(launchedTimer.Get()); }

    /**
     *
     *
     * <h2>Adds a {@link GamePieceOnFieldSimulation} to a {@link SimulatedArena} to Simulate the Game Piece After
     * Touch-Ground.</h2>
     *
     * <p>The added {@link GamePieceOnFieldSimulation} will have the initial velocity of the game piece projectile.
     *
     * <p>The game piece will start falling from mid-air until it touches the ground.
     *
     * <p>The added {@link GamePieceOnFieldSimulation} will always have collision space on the field, even before
     * touching the ground.
     *
     * @param simulatedArena the arena simulation to which the game piece will be added, usually obtained from
     *     {@link SimulatedArena#getInstance()}
     */
    void AddGamePieceAfterTouchGround(SimulatedArena& simulatedArena);

    /**
     *
     *
     * <h2>Check every {@link GamePieceProjectile} instance for available actions.</h2>
     *
     * <p>1. If a game piece {@link #hasHitTarget()}, remove it and run {@link #hitTargetCallBack} specified by
     * {@link #withHitTargetCallBack(Runnable)}
     *
     * <p>2. If a game piece {@link #hasHitGround()}, remove it and create a corresponding
     * {@link GamePieceOnFieldSimulation} using {@link #addGamePieceAfterTouchGround(SimulatedArena)}
     *
     * <p>3. If a game piece {@link #hasGoneOutOfField()}, remove it.
     */
    static void UpdateGamePieceProjectiles(SimulatedArena& simulatedArena, std::unordered_set<GamePieceProjectile> gamePieceProjectiles);

    // The rest are methods to configure a game piece projectile simulation
    /**
     *
     *
     * <h2>Configures the Game Piece Projectile to Automatically Become a {@link GamePieceOnFieldSimulation} Upon
     * Touching Ground.</h2>
     *
     * <p>This method configures the game piece projectile to transform into a {@link GamePieceOnFieldSimulation} when
     * it touches the ground.
     *
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     */
    constexpr GamePieceProjectile& EnableBecomesGamePieceOnFieldAfterTouchGround() {
        becomesGamePieceOnGroundAfterTouchGround = true;
        return *this;
    }

    /**
     *
     *
     * <h2>Configures the Game Piece Projectile to Disappear Upon Touching Ground.</h2>
     *
     * <p>Reverts the effect of {@link #enableBecomesGamePieceOnFieldAfterTouchGround()}.
     */
    constexpr GamePieceProjectile& DisableBecomesGamePieceOnFieldAfterTouchGround() {
        becomesGamePieceOnGroundAfterTouchGround = false;
        return *this;
    }

    /**
     *
     *
     * <h2>Sets a Target for the Game Projectile.</h2>
     *
     * <p>Configures the {@link #targetPositionSupplier} of this game piece projectile.
     *
     * <p>The method {@link #launch()} will estimate whether or not the game piece will hit the target.
     *
     * <p>After calling {@link #launch()}, {@link #hasHitTarget()} will indicate whether the game piece has already hit
     * the target.
     *
     * <p>Before calling this method, the target position is <code>0, 0, -100 (x,y,z)</code>, which the projectile will
     * never hit.
     *
     * @param targetPositionSupplier the position of the target, represented as a {@link Translation3d}
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     */
    inline GamePieceProjectile& WithTargetPosition(std::function<frc::Translation3d()> targetPositionSupplier) {
        this->targetPositionSupplier = targetPositionSupplier;
        return *this;
    }

    /**
     *
     *
     * <h2>Sets the Target Tolerance for the Game Projectile.</h2>
     *
     * <p>Configures the {@link #tolerance} for determining whether the game piece has hit the target. The tolerance
     * defines how close the projectile needs to be to the target for it to be considered a hit.
     *
     * <p>If this method is not called, the default tolerance is <code>0.2, 0.2, 0.2 (x,y,z)</code>
     *
     * @param tolerance the tolerance for the target, represented as a {@link Translation3d}
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     */
    constexpr GamePieceProjectile& WithTargetTolerance(frc::Translation3d tolerance) {
        this->tolerance = tolerance;
        return *this;
    }

    /**
     *
     *
     * <h2>Configures a callback to be executed when the game piece hits the target.</h2>
     *
     * <p>Sets the {@link #hitTargetCallBack} to Execute When the Game Piece Hits the Target.
     *
     * <p>The callback will be triggered when {@link #hasHitTarget()} becomes <code>true</code>.
     *
     * @param hitTargetCallBack the callback to run when the game piece hits the target
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     */
    inline GamePieceProjectile& WithHitTargetCallBack(std::function<void()> hitTargetCallBack) {
        this->hitTargetCallBack = hitTargetCallBack;
        return *this;
    }

    /**
     *
     *
     * <h2>Configures a Callback to Display the Trajectory of the Projectile When Launched.</h2>
     *
     * <p>Sets the {@link #projectileTrajectoryDisplayCallBackHitTarget} to be fed with data during the
     * {@link #launch()} method.
     *
     * <p>A {@link List} containing up to 50 {@link Pose3d} objects will be passed to the callback, representing the
     * future trajectory of the projectile.
     *
     * <p>This is usually for visualizing the trajectory of the projectile on a telemetry, like <a
     * href='https://github.com/Mechanical-Advantage/AdvantageScope'>Advantage Scope</a>
     *
     * @param projectileTrajectoryDisplayCallBack the callback that will receive the list of {@link Pose3d} objects
     *     representing the projectile's trajectory
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     */
    inline GamePieceProjectile& WithProjectileTrajectoryDisplayCallBack(
        std::function<void(std::vector<frc::Pose3d>)> projectileTrajectoryDisplayCallBack) {
        projectileTrajectoryDisplayCallBackHitTarget = projectileTrajectoryDisplayCallBack;
        projectileTrajectoryDisplayCallBackMiss = projectileTrajectoryDisplayCallBack;
        return *this;
    }

    /**
     *
     *
     * <h2>Configures a Callback to Display the Trajectory of the Projectile When Launched.</h2>
     *
     * <p>Sets the {@link #projectileTrajectoryDisplayCallBackHitTarget} to be fed with data during the
     * {@link #launch()} method.
     *
     * <p>A {@link List} containing up to 50 {@link Pose3d} objects will be passed to the callback, representing the
     * future trajectory of the projectile.
     *
     * <p>This is usually for visualizing the trajectory of the projectile on a telemetry, like <a
     * href='https://github.com/Mechanical-Advantage/AdvantageScope'>Advantage Scope</a>
     *
     * @param projectileTrajectoryDisplayCallBackHitTarget the callback that will receive the list of {@link Pose3d}
     *     objects representing the projectile's trajectory, called if the projectile will hit the target on its path
     * @param projectileTrajectoryDisplayCallBackHitTargetMiss the callback that will receive the list of {@link Pose3d}
     *     objects representing the projectile's trajectory, called if the projectile will be off the target
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     */
    inline GamePieceProjectile& WithProjectileTrajectoryDisplayCallBack(
        std::function<void(std::vector<frc::Pose3d>)> projectileTrajectoryDisplayCallBackHitTarget,
        std::function<void(std::vector<frc::Pose3d>)> projectileTrajectoryDisplayCallBackHitTargetMiss) {
        this->projectileTrajectoryDisplayCallBackHitTarget = projectileTrajectoryDisplayCallBackHitTarget;
        this->projectileTrajectoryDisplayCallBackMiss = projectileTrajectoryDisplayCallBackHitTargetMiss;
        return *this;
    }

    /**
     *
     *
     * <h2>Configures the Height at Which the Projectile Is Considered to Be Touching Ground.</h2>
     *
     * <p>Sets the {@link #heightAsTouchGround}, defining the height at which the projectile is considered to have
     * landed.
     *
     * <p>When the game piece is below this height, it will either be deleted or, if configured, transformed into a
     * {@link GamePieceOnFieldSimulation} using {@link #enableBecomesGamePieceOnFieldAfterTouchGround()}.
     *
     * @param heightAsTouchGround the height (in meters) at which the projectile is considered to touch the ground
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     */
    constexpr GamePieceProjectile& WithTouchGroundHeight(units::meter_t heightAsTouchGround) {
        this->heightAsTouchGround = heightAsTouchGround;
        return *this;
    }

    constexpr std::string GetType() const override { return gamePieceType; }

    constexpr bool IsGrounded() const override { return false; }

    inline void TriggerHitTargetCallBack() { hitTargetCallBack(); }

    /**
     *
     *
     * <h2>Registers a function that will be called whenever this game piece interacts with a goal object or collides
     * with the specified target.</h2>
     *
     * Note: Certain goals like the rebuilt hub or the reefscape processor appear to reintroduce scored pieces into the
     * field. However these pieces are actually new objects and so will not save any call back functions.
     *
     * @param hitTargetCallBack The call back to be set.
     */
    void SetHitTargetCallBack(std::function<void()> hitTargetCallBack) { this->hitTargetCallBack = hitTargetCallBack; }

    /**
     * This value may seem unusual compared to the standard 9.8 m/s² for gravity. However, through experimentation, it
     * appears more realistic in our simulation, possibly due to the ignoring of air drag.
     */
    units::meters_per_second_squared_t GRAVITY = 11_mps_sq;

    std::string gamePieceType;

   protected:
    /**
     *
     *
     * <h2>Calculates the Projectile's Position at a Given Time.</h2>
     *
     * <p>This method calculates the position of the projectile using the physics formula for projectile motion.
     *
     * @param t the time elapsed after the launch of the projectile, in seconds
     * @return the calculated position of the projectile at time <code>t</code> as a {@link Translation3d} object
     */
    constexpr frc::Translation3d GetPositionAtTime(units::second_t t) const {
        units::meter_t height = initialHeight + initialVerticalSpeed * t - 1.0 / 2 * GRAVITY * t * t;
        frc::Translation2d current2dPosition = initialPosition + frc::Translation2d{initialLaunchingVelocity * t.value()};
        return {current2dPosition.X(), current2dPosition.Y(), height};
    }

    // Properties of the game piece projectile:
    GamePieceOnFieldSimulation::GamePieceInfo info;
    frc::Translation2d initialPosition;
    Eigen::Vector2d initialLaunchingVelocity;
    units::meter_t initialHeight;
    units::meters_per_second_t initialVerticalSpeed;
    frc::Rotation3d gamePieceRotation;
    frc::Timer launchedTimer;

    // Optional properties of the game piece, used if we want it to become a
    // GamePieceOnFieldSimulation upon touching ground:
    bool becomesGamePieceOnGroundAfterTouchGround = false;

   private:
    /**
     *
     *
     * <h2>Calculates the Initial Velocity of the Game Piece Projectile in the X-Y Plane.</h2>
     *
     * <p>This method calculates the initial velocity of the game piece projectile, accounting for the chassis's
     * translational and rotational motion as well as the shooter's ground speed.
     *
     * @param shooterPositionOnRobot the translation of the shooter on the robot, in the robot's frame of reference
     * @param chassisSpeeds the speeds of the chassis when the game piece is launched, including translational and
     *     rotational velocities
     * @param chassisFacing the direction the chassis is facing at the time of the launch
     * @param groundSpeedMPS the ground component of the projectile's initial velocity, provided as a scalar in meters
     *     per second (m/s)
     * @return the calculated initial velocity of the projectile as a {@link Translation2d} in meters per second
     */
    static Eigen::Vector2d CalculateInitialProjectileVelocity(frc::Translation2d shooterPositionOnRobot, frc::ChassisSpeeds chassisSpeeds,
                                                              frc::Rotation2d chassisFacing, units::meters_per_second_t groundSpeed);

    bool IsOutOfField(units::second_t time) const;

    /**
     *
     *
     * <h2>Calculates the Projectile's Velocity at a Given Time.</h2>
     *
     * <p>This method calculates the 3d velocity of the projectile using the physics formula for projectile motion.
     *
     * @param t the time elapsed after the launch of the projectile, in seconds
     * @return a {@link Translation3d} object representing the calculated 3d velocity of the projectile at time <code>t
     *     </code>, in meters per second
     */
    inline Eigen::Vector3d GetVelocityAtTime(units::second_t t) const {
        units::meters_per_second_t verticalVelocity = initialVerticalSpeed - GRAVITY * t;
        return {initialLaunchingVelocity.x(), initialLaunchingVelocity.y(), verticalVelocity.value()};
    }

    /**
     *
     *
     * <h2>Visualizes the Projectile Flight Trajectory.</h2>
     *
     * <p>Optionally, this callback will be used to visualize the projectile flight trajectory in a telemetry system,
     * such as <a href='https://github.com/Mechanical-Advantage/AdvantageScope'>Advantage Scope</a>.
     */
    std::function<void(std::vector<frc::Pose3d>)> projectileTrajectoryDisplayCallBackHitTarget;
    std::function<void(std::vector<frc::Pose3d>)> projectileTrajectoryDisplayCallBackMiss;

    // Optional properties of the game piece, used if we want it to have a target:
    frc::Translation3d tolerance{0.2_m, 0.2_m, 0.2_m};
    std::function<frc::Translation3d()> targetPositionSupplier = [] { return frc::Translation3d{0_m, 0_m, -100_m}; };
    std::function<void()> hitTargetCallBack;
    units::meter_t heightAsTouchGround = 0.5_m;

    /**
     *
     *
     * <h2>Time to Hit the Desired Target.</h2>
     *
     * <p>This value represents the amount of time it takes for the projectile to hit the desired target, calculated
     * when the {@link #launch()} method is called.
     *
     * <p>Determines the results of {@link #hasHitTarget()} and {@link #willHitTarget()}
     *
     * <p>If the projectile never hits the target, or if there is no target, this value remains <code>
     * -1</code>.
     */
    std::optional<units::second_t> calculatedHitTargetTime;

    bool hitTargetCallBackCalled = false;
};

}  // namespace maplesim
