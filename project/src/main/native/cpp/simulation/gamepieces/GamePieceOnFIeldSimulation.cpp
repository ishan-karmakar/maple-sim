#include <btBulletDynamicsCommon.h>
#include "maplesim/simulation/gamepieces/GamePieceOnFieldSimulation.h"

using namespace maplesim;

GamePieceOnFieldSimulation::GamePieceOnFieldSimulation(const GamePieceInfo& info, std::function<units::meter_t()> zPositionSupplier,
                                                       const frc::Pose2d& initialPose, const Eigen::Vector2d& initialVelocity)
      : type{info.type}, zPositionSupplier{zPositionSupplier}, collisionShape{info.shape} {
    compoundShape = std::make_unique<btCompoundShape>();
    compoundShape->addChildShape(btTransform::getIdentity(), collisionShape.get());

    btVector3 inertia;
    if (info.gamePieceMass)
        compoundShape->calculateLocalInertia(info.gamePieceMass.value(), inertia);

    btTransform initialTransform = btTransform::getIdentity();
    initialTransform.setOrigin({initialPose.X().value(), initialPose.Y().value(), 0});
    btQuaternion quat;
    quat.setRotation({0, 0, 1}, initialPose.Rotation().Radians().value());
    initialTransform.setRotation(quat);

    motionState = std::make_unique<btDefaultMotionState>(initialTransform);

    btRigidBody::btRigidBodyConstructionInfo rbInfo{info.gamePieceMass.value(), motionState.get(), compoundShape.get(), inertia};

    rigidBody = std::make_unique<btRigidBody>(rbInfo);

    rigidBody->setFriction(COEFFICIENT_OF_FRICTION);
    rigidBody->setRestitution(info.coefficientOfRestitution);

    rigidBody->setDamping(info.linearDamping, info.angularDamping);

    rigidBody->setCcdMotionThreshold(0.001);
    rigidBody->setCcdSweptSphereRadius(0.05);

    if (initialVelocity.norm())
        rigidBody->setLinearVelocity({initialVelocity.x(), initialVelocity.y(), 0});
}

GamePieceOnFieldSimulation::~GamePieceOnFieldSimulation() = default;

void GamePieceOnFieldSimulation::SetVelocity(frc::ChassisSpeeds speeds) {
    rigidBody->setLinearVelocity({speeds.vx.value(), speeds.vy.value(), 0});
    rigidBody->setAngularVelocity({0, 0, speeds.omega.value()});
    rigidBody->activate(true);
}

frc::Pose2d GamePieceOnFieldSimulation::GetPoseOnField() const {
    btTransform transform = rigidBody->getWorldTransform();
    btQuaternion quat = transform.getRotation();

    return {
        units::meter_t{transform.getOrigin().x()}, units::meter_t{transform.getOrigin().y()},
        units::radian_t{std::atan2(2 * (quat.w() * quat.z() + quat.x() * quat.y()), 1 - 2 * (quat.y() * quat.y() + quat.z() * quat.z()))}};
}

Eigen::Vector3d GamePieceOnFieldSimulation::GetVelocity3dMPS() const {
    btVector3 linearVel = rigidBody->getLinearVelocity();
    return {linearVel.x(), linearVel.y(), linearVel.z()};
}
