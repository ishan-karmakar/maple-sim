#include "maplesim/simulation/gamepieces/GamePieceProjectile.h"

using namespace maplesim;

void GamePieceProjectile::Launch() {
    int maxIterations = 100;
    units::second_t stepSeconds = 0.02_s;
    std::vector<frc::Pose3d> trajectoryPoints;

    for (int i = 0; i < maxIterations; i++) {
        units::second_t t = i * stepSeconds;
        frc::Translation3d currentPosition = GetPositionAtTime(t);
        trajectoryPoints.emplace_back(currentPosition, gamePieceRotation);

        if (currentPosition.Z() < heightAsTouchGround && t * GRAVITY > initialVerticalSpeed)
            break;
        if (IsOutOfField(t))
            break;
        frc::Translation3d displacementToTarget = targetPositionSupplier() - currentPosition;
        if (units::math::abs(displacementToTarget.X()) < tolerance.X() && units::math::abs(displacementToTarget.Y()) < tolerance.Y() &&
            units::math::abs(displacementToTarget.Z()) < tolerance.Z()) {
            calculatedHitTargetTime = t;
            break;
        }
    }
    if (WillHitTarget())
        projectileTrajectoryDisplayCallBackHitTarget(trajectoryPoints);
    else
        projectileTrajectoryDisplayCallBackMiss(trajectoryPoints);
    hitTargetCallBackCalled = false;

    launchedTimer.Start();
}

bool GamePieceProjectile::IsOutOfField(units::second_t time) const {
    frc::Translation3d position = GetPositionAtTime(time);
    units::meter_t EDGE_TOLERANCE = 2_m;
    // FIXME: Set to use field layout constants
    return position.X() < -EDGE_TOLERANCE || position.X() > 16.54_m + EDGE_TOLERANCE || position.Y() < -EDGE_TOLERANCE ||
           position.Y() > 8.21_m + EDGE_TOLERANCE;
}

Eigen::Vector2d GamePieceProjectile::CalculateInitialProjectileVelocity(frc::Translation2d shooterPositionOnRobot,
                                                                        frc::ChassisSpeeds chassisSpeeds, frc::Rotation2d chassisFacing,
                                                                        units::meters_per_second_t groundSpeed) {
    Eigen::Vector2d chassisTranslationalVelocity{chassisSpeeds.vx.value(), chassisSpeeds.vy.value()};
    Eigen::Vector2d shooterGroundVelocityDueToChassisRotation =
        Eigen::Rotation2Dd{chassisFacing.Radians().value()} * shooterPositionOnRobot.ToVector();
    shooterGroundVelocityDueToChassisRotation = Eigen::Rotation2Dd{std::numbers::pi / 2} * shooterGroundVelocityDueToChassisRotation;
    shooterGroundVelocityDueToChassisRotation *= chassisSpeeds.omega.value();
    Eigen::Vector2d shooterGroundVelocity = chassisTranslationalVelocity + shooterGroundVelocityDueToChassisRotation;

    Eigen::Vector2d facingDir = Eigen::Rotation2Dd{chassisFacing.Radians().value()} * Eigen::Vector2d::UnitX();
    return shooterGroundVelocity + facingDir * groundSpeed.value();
}
