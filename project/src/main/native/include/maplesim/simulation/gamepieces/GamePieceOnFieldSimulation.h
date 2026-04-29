#pragma once
#include <units/mass.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include "maplesim/simulation/gamepieces/GamePiece.h"

class btRigidBody;
class btCollisionShape;
class btCompoundShape;
struct btDefaultMotionState;

namespace maplesim {

/**
 *
 *
 * <h1>Simulates a Game Piece on the Field.</h1>
 *
 * <p>This class simulates a game piece on the field, which has a collision space and interacts with other objects.
 *
 * <p>Game pieces can be "grabbed" by an {@link IntakeSimulation}.
 *
 * <p>For the simulation to actually run, every instance must be added to a
 * {@link org.ironmaple.simulation.SimulatedArena} through
 * {@link SimulatedArena#addGamePiece(GamePieceOnFieldSimulation)}.
 */
class GamePieceOnFieldSimulation : GamePiece {
   public:
    struct GamePieceInfo {
        std::string type;
        std::shared_ptr<btCollisionShape> shape;
        units::meter_t gamePieceHeight;
        units::kilogram_t gamePieceMass;
        double linearDamping;
        double angularDamping;
        double coefficientOfRestitution;
    };

    static constexpr double COEFFICIENT_OF_FRICTION = 0.8;
    static constexpr units::meters_per_second_t MINIMUM_BOUNDING_VELOCITY = 0.2_mps;

    /**
     *
     *
     * <h2>Creates a Game Piece on the Field with Fixed Height.</h2>
     *
     * @param info info about the game piece type
     * @param initialPose the initial position of the game piece on the field
     */
    inline GamePieceOnFieldSimulation(const GamePieceInfo& info, const frc::Pose2d& initialPose)
          : GamePieceOnFieldSimulation{info, [=] { return info.gamePieceHeight / 2; }, initialPose, {}} {}

    /**
     *
     *
     * <h2>Creates a Game Piece on the Field with Custom Height Supplier and Initial Velocity.</h2>
     *
     * @param info info about the game piece type
     * @param zPositionSupplier a supplier that provides the current Z-height of the game piece
     * @param initialPose the initial position of the game piece on the field
     * @param initialVelocityMPS the initial velocity of the game piece, in meters per second
     */
    GamePieceOnFieldSimulation(const GamePieceInfo& info, std::function<units::meter_t()> zPositionSupplier, const frc::Pose2d& initialPose,
                               const Eigen::Vector2d& initialVelocity);

    /**
     *
     *
     * <h2>Sets the world velocity of this game piece.</h2>
     *
     * @param chassisSpeedsWorldFrame the speeds of the game piece
     */
    void SetVelocity(frc::ChassisSpeeds);

    /**
     *
     *
     * <h2>Obtains the 2d position of the game piece</h2>
     *
     * @return the 2d position of the game piece
     */
    frc::Pose2d GetPoseOnField() const;

    /**
     *
     *
     * <h2>Obtains a 3d pose of the game piece.</h2>
     *
     * <p>The 3d position is calculated from both the {@link #getPoseOnField()} and {@link #zPositionSupplier}
     *
     * @return the 3d position of the game piece
     */
    inline frc::Pose3d GetPose3d() const {
        frc::Pose2d pose2d = GetPoseOnField();
        return frc::Pose3d{pose2d.X(), pose2d.Y(), zPositionSupplier(), frc::Rotation3d{pose2d.Rotation()}};
    }

    void OnIntake(std::string intakeTargetGamePieceType);

    constexpr std::string GetType() const override { return type; }

    Eigen::Vector3d GetVelocity3dMPS() const override;

    constexpr bool IsGrounded() const override { return true; }

    constexpr void TriggerHitTargetCallback() override {}

   private:
    /**
     *
     *
     * <h2>Supplier of the Current Z-Pose (Height) of the Game Piece.</h2>
     *
     * <p>Normally, the height is fixed at half the thickness of the game piece to simulate it being "on the ground."
     *
     * <p>If the game piece is flying at a low height, the height is calculated using the law of free-fall.
     */
    std::function<units::meter_t()> zPositionSupplier;

    /**
     *
     *
     * <h2>The Type of the Game Piece.</h2>
     *
     * <p>Affects the result of {@link SimulatedArena#getGamePiecesPosesByType(String)}.
     */
    std::string type;

    std::unique_ptr<btRigidBody> rigidBody;
    std::shared_ptr<btCollisionShape> collisionShape;
    std::unique_ptr<btCompoundShape> compoundShape;
    std::unique_ptr<btDefaultMotionState> motionState;
};

}  // namespace maplesim
