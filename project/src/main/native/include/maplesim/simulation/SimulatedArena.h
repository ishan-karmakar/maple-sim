#pragma once
#include <unordered_map>
#include <unordered_set>
#include <networktables/DoubleTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include "maplesim/simulation/drivesims/AbstractDriveTrainSimulation.h"
#include "maplesim/simulation/gamepieces/GamePieceProjectile.h"

class btDiscreteDynamicsWorld;
class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btBroadphaseInterface;
class btSequentialImpulseConstraintSolver;

namespace maplesim {

/**
 *
 *
 * <h1>Abstract Simulation World</h1>
 *
 * <p>Check <a href='https://shenzhen-robotics-alliance.github.io/maple-sim/using-the-simulated-arena/'>Online
 * Documentation</a>
 *
 * <h2>The heart of the simulator.</h2>
 *
 * <p>This class cannot be instantiated directly; it must be created as a specific season's arena.
 *
 * <p>The default instance can be obtained using the {@link #getInstance()} method.
 *
 * <p>Simulates all interactions within the arena field.
 *
 * <h2>The following objects can be added to the simulation world and will interact with each other: </h2>
 *
 * <ul>
 *   <li>{@link AbstractDriveTrainSimulation}: Represents abstract drivetrain simulations with collision detection.
 *   <li>{@link GamePieceOnFieldSimulation}: Represents abstract game pieces with collision detection.
 *   <li>{@link IntakeSimulation}: Represents an intake simulation that responds to contact with
 *       {@link GamePieceOnFieldSimulation}.
 * </ul>
 */
class SimulatedArena {
   public:
    /**
     *
     *
     * <h2>Represents a custom simulation to be updated during each simulation sub-tick.</h2>
     *
     * <p>This allows you to register custom actions that will be executed at a high frequency during each simulation
     * sub-tick. This is useful for tasks that need to be updated multiple times per simulation cycle.
     *
     * <p>Examples of how this method is used:
     *
     * <ul>
     *   <li>Pulling encoder values for high-frequency odometry updates.
     *   <li>Adding custom simulation objects or handling events in the simulated arena.
     * </ul>
     */
    class Simulatable {
       public:
        /**
         * Called in {@link #simulationSubTick(int)}.
         *
         * @param subTickNum the number of this sub-tick (counting from 0 in each robot period)
         */
        virtual void SimulationSubTick(int subTickNum) = 0;
    };

    /**
     *
     *
     * <h1>Represents an Abstract Field Map</h1>
     *
     * <p>Stores the layout of obstacles and game pieces.
     *
     * <p>For each season-specific subclass of {@link SimulatedArena}, there should be a corresponding subclass of this
     * class to store the field map for that specific season's game.
     */
    class FieldMap {
       public:
        ~FieldMap();

        std::vector<std::unique_ptr<btRigidBody>> obstacles;

       protected:
        void AddBorderLine(const frc::Translation2d& startingPoint, const frc::Translation2d& endingPoint);
        void AddRectangularObstacle(units::meter_t width, units::meter_t height, const frc::Pose2d& absolutePositionOnField);
        void AddCustomObstacle(btCollisionShape* shape, const frc::Pose2d& absolutePositionOnField);

       private:
        static std::unique_ptr<btRigidBody> CreateObstacle(btCollisionShape* shape);
    };

    /**
     *
     *
     * <h2>Gets/Creates the Default Simulation World</h2>
     *
     * <p>Multiple instances of {@link SimulatedArena} can exist elsewhere.
     *
     * @return the main simulation arena instance
     * @throws IllegalStateException if the method is call when running on a real robot
     */
    static SimulatedArena& GetInstance();

    /**
     *
     *
     * <h2>Overrides the Default Simulation World</h2>
     *
     * <p>Overrides the return value of {@link #getInstance()}
     *
     * <p>This method allows simulating an arena from a different year or a custom field.
     *
     * <p>Currently, only the 2024 arena is supported, so avoid calling this method for now.
     *
     * @param newInstance the new simulation arena instance to override the current one
     */
    static inline void OverrideInstance(SimulatedArena&& newInstance) { GetInstance() = std::move(newInstance); }

    static constexpr int GetSimulationSubTicksIn1Period() { return SIMULATION_SUB_TICKS_IN_1_PERIOD; }

    static constexpr units::second_t GetSimulationDt() { return SIMULATION_DT; }

    /**
     *
     *
     * <h2>Returns the score of the specified team.</h2>
     *
     * @param isBlue The team to return the score of as a bool.
     * @return The score of the specified team.
     */
    constexpr int GetScore(bool isBlue) const { return isBlue ? blueScore : redScore; }

    /**
     *
     *
     * <h2>Returns the score of the specified team.</h2>
     *
     * @param allianceColor The team to return the score of as a Alliance enum.
     * @return The score of the specified team.
     */
    constexpr int GetScore(frc::DriverStation::Alliance allianceColor) const {
        return GetScore(allianceColor == frc::DriverStation::kBlue);
    }

    /**
     *
     *
     * <h2>Adds to the score of the specified team</h2>
     *
     * @param isBlue Wether to add to the blue or red team score.
     * @param toAdd How many points to add.
     */
    void AddToScore(bool isBlue, int toAdd);

    /**
     *
     *
     * <h2>Overrides the Timing Configurations of the Simulations.</h2>
     *
     * <h4>If Using <a href='https://github.com/Mechanical-Advantage/AdvantageKit'>Advantage-Kit</a>: DO NOT CHANGE THE
     * DEFAULT TIMINGS</h4>
     *
     * <p>Changes apply to every instance of {@link SimulatedArena}.
     *
     * <p>The new configuration will take effect the next time {@link SimulatedArena#simulationPeriodic()} is called on
     * an instance.
     *
     * <p>It is recommended to call this method before the first call to {@link SimulatedArena#simulationPeriodic()} of
     * any instance.
     *
     * <p>It is also recommended to keep the simulation frequency above 200 Hz for accurate simulation results.
     *
     * @param robotPeriod the time between two calls of {@link #simulationPeriodic()}, usually obtained from
     *     {@link TimedRobot#getPeriod()}
     * @param simulationSubTicksPerPeriod the number of Iterations, or {@link #simulationSubTick(int)} that the
     *     simulation runs per each call to {@link #simulationPeriodic()}
     */
    static inline void OverrideSimulationTimings(units::second_t robotPeriod, int simulationSubTicksPerPeriod) {
        SIMULATION_SUB_TICKS_IN_1_PERIOD = simulationSubTicksPerPeriod;
        SIMULATION_DT = robotPeriod / SIMULATION_SUB_TICKS_IN_1_PERIOD;
    }

    /**
     *
     *
     * <h2>Registers a custom simulation.</h2>
     *
     * @param simulatable the custom simulation to register
     */
    inline void AddCustomSimulation(std::unique_ptr<Simulatable> simulatable) { customSimulations.push_back(std::move(simulatable)); }

    /**
     *
     *
     * <h2>Tells the arena to start publishing the match breakdown data to network tables</h2>
     */
    constexpr void EnableBreakdownPublishing() { shouldPublishMatchBreakdown = true; }

    /**
     *
     *
     * <h2>Tells the arena to stop publishing the match breakdown data to network tables</h2>
     */
    constexpr void DisableBreakdownPublishing() { shouldPublishMatchBreakdown = false; }

    /**
     *
     *
     * <h2>replaces or adds a value to the match scoring breakdown published to network tables</h2>
     *
     * @param isBlueTeam Wether to add to the blue teams match breakdown or the red teams match breakdown
     * @param valueKey The name of the value to be added
     * @param value The value to be added
     */
    void ReplaceValueInMatchBreakdown(bool isBlueTeam, std::string valueKey, double value);

    /**
     *
     *
     * <h2>Defaults a value to 0 and creates it in the match breakdown. This is useful on startup to make sure all match
     * breakdown values display before they are first updated</h2>
     *
     * @param valueKey The key to add to match breakdown
     */
    inline void SetupValueForMatchBreakdown(std::string valueKey) {
        ReplaceValueInMatchBreakdown(true, valueKey, 0);
        ReplaceValueInMatchBreakdown(false, valueKey, 0);
    }

    /**
     *
     *
     * <h2>Adds too a value in the scoring breakdown. If value does not already exist in the scoring breakdown it will
     * be defaulted to 0 and then added too
     *
     * @param isBlueTeam Wether to add to the blue teams match breakdown or the red teams match breakdown
     * @param ValueKey The name of the value to be added too
     * @param toAdd how much to be added to specified value
     */
    void AddValueToMatchBreakdown(bool isBlueTeam, std::string valueKey, double toAdd);

    /**
     *
     *
     * <h2>Registers a {@link GamePieceProjectile} to the Simulation and Launches It.</h2>
     *
     * <p>Calls to {@link GamePieceProjectile#launch()}, which will launch the game piece immediately.
     *
     * @param gamePieceProjectile the projectile to be registered and launched in the simulation
     */
    inline void AddGamePieceProjectile(std::unique_ptr<GamePieceProjectile> gamePieceProjectile) {
        gamePieceProjectile->Launch();
        gamePieces.emplace(gamePieceProjectile);
    }

    /**
     *
     *
     * <h2>Removes a {@link GamePieceOnFieldSimulation} from the Simulation.</h2>
     *
     * <p>Removes the game piece from the physics world and the simulation's game piece collection.
     *
     * @param gamePiece the game piece to be removed from the simulation
     * @return <code>true</code> if this set contained the specified element
     */
    bool RemoveGamePiece(std::shared_ptr<GamePieceOnFieldSimulation> gamePiece) {
        physicsWorld->removeRigidBody(gamePiece.get());
    }

    /** Whether to allow the simulation to run a real robot This feature is HIGHLY RECOMMENDED to be turned OFF */
    static bool ALLOW_CREATION_ON_REAL_ROBOT;

    std::unordered_map<std::string, double> redScoringBreakdown;
    std::unordered_map<std::string, double> blueScoringBreakdown;

    std::shared_ptr<nt::NetworkTable> redTable =
        nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard/MapleSim/MatchData/Breakdown/Red Alliance");
    std::shared_ptr<nt::NetworkTable> blueTable =
        nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard/MapleSim/MatchData/Breakdown/Blue Alliance");
    std::shared_ptr<nt::NetworkTable> genericInfoTable =
        nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard/MapleSim/MatchData/Breakdown");

    nt::DoublePublisher matchClockPublisher = genericInfoTable->GetDoubleTopic("Match Clock").Publish();

    static nt::BooleanPublisher resetFieldPublisher;
    static nt::BooleanSubscriber resetFieldSubscriber;

   protected:
    /**
     *
     *
     * <h2>Constructs a new simulation arena with the specified field map of obstacles.</h2>
     *
     * <p>This constructor initializes a physics world with zero gravity and adds the provided obstacles to the world.
     *
     * <p>It also sets up the collections for drivetrain simulations, game pieces, projectiles, and intake simulations.
     *
     * @param obstaclesMap the season-specific field map containing the layout of obstacles for the simulation
     */
    SimulatedArena(FieldMap obstaclesMap);

    /**
     *
     *
     * <h2>Publishes the match breakdown data to network tables</h2>
     */
    void PublishBreakdown();

    int redScore = 0;
    int blueScore = 0;
    frc::Timer matchClock;

    std::unordered_map<std::string, nt::DoublePublisher> redPublishers;
    std::unordered_map<std::string, nt::DoublePublisher> bluePublishers;

    std::unique_ptr<btDiscreteDynamicsWorld> physicsWorld;
    std::unordered_set<std::unique_ptr<AbstractDriveTrainSimulation>> driveTrainSimulations;

    std::unordered_set<std::unique_ptr<GamePiece>> gamePieces;
    std::vector<std::unique_ptr<Simulatable>> customSimulations;

   private:
    /** The number of sub-ticks the simulator will run in each robot period. */
    static int SIMULATION_SUB_TICKS_IN_1_PERIOD;
    /** The period length of each sub-tick, in seconds. */
    static units::second_t SIMULATION_DT;

    bool shouldPublishMatchBreakdown = true;

    std::unique_ptr<btDefaultCollisionConfiguration> collisionConfiguration;
    std::unique_ptr<btCollisionDispatcher> collisionDispatcher;
    std::unique_ptr<btBroadphaseInterface> broadphase;
    std::unique_ptr<btSequentialImpulseConstraintSolver> constraintSolver;

    // Intake simulations
};

}  // namespace maplesim
