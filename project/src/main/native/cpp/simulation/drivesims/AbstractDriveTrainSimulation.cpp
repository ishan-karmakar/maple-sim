#include <btBulletDynamicsCommon.h>
#include "maplesim/simulation/drivesims/AbstractDriveTrainSimulation.h"

using namespace maplesim;

AbstractDriveTrainSimulation::AbstractDriveTrainSimulation(const DriveTrainSimulationConfig& config, const frc::Pose2d& initialPose)
      : config{config} {
    btVector3 halfExtents{config.bumperLengthX.value() / 2, config.bumperWidthY.value() / 2, 0.1};
    shape = std::make_unique<btBoxShape>(halfExtents);

    btVector3 localInertia;
    if (config.robotMass)
        shape->calculateLocalInertia(config.robotMass.value(), localInertia);

    btTransform initialTransform = btTransform::getIdentity();

    motionState = std::make_unique<btDefaultMotionState>(initialTransform);

    btRigidBody::btRigidBodyConstructionInfo rbInfo{config.robotMass.value(), motionState.get(), shape.get(), localInertia};

    rbInfo.m_friction = BUMPER_COEFFICIENT_OF_FRICTION;
    rbInfo.m_restitution = BUMPER_COEFFICIENT_OF_FRICTION;
    rbInfo.m_linearDamping = 0.1;
    rbInfo.m_angularDamping = 0.1;

    body = std::make_unique<btRigidBody>(rbInfo);

    body->setLinearFactor({1, 1, 0});
    body->setAngularFactor({0, 0, 1});

    SetSimulationWorldPose(initialPose);
}

void AbstractDriveTrainSimulation::SetSimulationWorldPose(const frc::Pose2d& robotPose) {
    btTransform trans = btTransform::getIdentity();
    trans.setOrigin({robotPose.X().value(), robotPose.Y().value(), 0});

    trans.setRotation({{0, 0, 1}, robotPose.Rotation().Radians().value()});

    body->setWorldTransform(trans);
    motionState->setWorldTransform(trans);

    body->setLinearVelocity({0, 0, 0});
    body->setAngularVelocity({0, 0, 0});
}

void AbstractDriveTrainSimulation::SetRobotSpeeds(const frc::ChassisSpeeds& speeds) {
    frc::Rotation2d currentRotation = GetSimulatedDriveTrainPose().Rotation();
    frc::ChassisSpeeds fieldRelative = frc::ChassisSpeeds::FromRobotRelativeSpeeds(speeds, currentRotation);

    body->setLinearVelocity({fieldRelative.vx.value(), fieldRelative.vy.value(), 0});
    body->setAngularVelocity({0, 0, speeds.omega.value()});
}

frc::Pose2d AbstractDriveTrainSimulation::GetSimulatedDriveTrainPose() const {
    btTransform trans;
    body->getMotionState()->getWorldTransform(trans);

    btScalar roll, pitch, yaw;
    trans.getBasis().getEulerZYX(yaw, pitch, roll);
    return {units::meter_t{trans.getOrigin().getX()}, units::meter_t{trans.getOrigin().getY()}, units::radian_t{yaw}};
}

frc::ChassisSpeeds AbstractDriveTrainSimulation::GetDriveTrainSimulatedChassisSpeedsFieldRelative() const {
    btVector3 linVel = body->getLinearVelocity();
    btVector3 angVel = body->getAngularVelocity();

    return {units::meters_per_second_t{linVel.x()}, units::meters_per_second_t{linVel.y()}, units::radians_per_second_t{angVel.z()}};
}

frc::ChassisSpeeds AbstractDriveTrainSimulation::GetDriveTrainSimulatedChassisSpeedsRobotRelative() const {
    frc::ChassisSpeeds fieldSpeeds = GetDriveTrainSimulatedChassisSpeedsFieldRelative();
    return frc::ChassisSpeeds::FromFieldRelativeSpeeds(fieldSpeeds, GetSimulatedDriveTrainPose().Rotation());
}
