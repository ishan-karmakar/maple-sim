#pragma once
#include <frc/geometry/Pose3d.h>

namespace maplesim {

/**
 *
 *
 * <h1>Interface for all Game pieces.</h1>
 *
 * <p>This class contains basic functions all game pieces will have so that game pieces of different types can be used
 * interchangeably in collision detection.
 */
class GamePiece {
   public:
    /**
     *
     *
     * <h2>Gives the pose3d of a game piece.</h2>
     *
     * @return The pose of this piece as a Pose3d.
     */
    virtual frc::Pose3d GetPose3d() const = 0;

    /**
     *
     *
     * <h2>Gives the string type of the current game piece.</h2>
     *
     * @return The game piece string type (ex "Coral", "Algae", "Note").
     */
    virtual std::string GetType() const = 0;

    /**
     *
     *
     * <h2>Gives the velocity of the game piece.</h2>
     *
     * For grounded game pieces the z access velocity does not exist and so will be set to 0 automatically.
     *
     * @return The velocity of the game piece as a Translation3d.
     */
    virtual Eigen::Vector3d GetVelocity3dMPS() const = 0;

    /**
     *
     *
     * <h2>Internal function called to trigger a callback whenever this game piece is scored in a goal </h2>
     */
    virtual void TriggerHitTargetCallback() = 0;

    /**
     *
     *
     * <h2>Gives wether or not the piece is "grounded".</h2>
     *
     * A grounded piece is likely a child of {@link GamePieceOnFieldSimulation} while a non grounded piece is likely a
     * child of{@link GamePieceProjectile}.
     *
     * @return wether or not the piece is grounded.
     */
    virtual bool IsGrounded() const = 0;
};

}  // namespace maplesim
