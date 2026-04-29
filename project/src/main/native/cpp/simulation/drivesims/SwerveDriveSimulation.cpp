#include <btBulletDynamicsCommon.h>
#include <Eigen/Geometry>
#include "maplesim/simulation/SimulatedArena.h"
#include "maplesim/simulation/drivesims/SwerveDriveSimulation.h"

using namespace maplesim;

SwerveDriveSimulation::SwerveDriveSimulation(const DriveTrainSimulationConfig& config, const frc::Pose2d& initialPose)
      : AbstractDriveTrainSimulation{config, initialPose},
        gyroSimulation{config.gyroSimulationFactory()},
        moduleTranslations{config.moduleTranslations},
        kinematics{moduleTranslations},
        gravityForceOnEachModule{config.robotMass * 1_SG / moduleTranslations.size()} {
    for (const auto& factory : config.swerveModuleSimulationFactories)
        moduleSimulations.push_back(factory());

    body->setDamping(1.4, 1.4);
}

void SwerveDriveSimulation::SimulationSubTick() {
    SimulateChassisFrictionForce();
    SimulateChassisFrictionTorque();
    SimulateModulePropellingForces();

    gyroSimulation.UpdateSimulationSubTick(units::radians_per_second_t{body->getAngularVelocity().z()});
}

void SwerveDriveSimulation::SimulateChassisFrictionForce() {
    frc::ChassisSpeeds moduleSpeeds = GetModuleSpeeds();

    frc::ChassisSpeeds diffRobotRelative = moduleSpeeds - GetDriveTrainSimulatedChassisSpeedsRobotRelative();

    frc::Rotation2d robotRotation = GetSimulatedDriveTrainPose().Rotation();

    Eigen::Vector2d floorAndModuleDiffFieldRel =
        Eigen::Rotation2Dd{robotRotation.Radians().value()} * Eigen::Vector2d{diffRobotRelative.vx.value(), diffRobotRelative.vy.value()};

    double FRICTION_FORCE_GAIN = 3;
    units::newton_t totalGrippingForce = moduleSimulations[0].config.GetGrippingForce(gravityForceOnEachModule) * moduleSimulations.size();

    units::newton_t forceMag =
        units::math::min(FRICTION_FORCE_GAIN * totalGrippingForce * floorAndModuleDiffFieldRel.norm(), totalGrippingForce);
    frc::Rotation2d forceDir{units::radian_t{std::atan2(floorAndModuleDiffFieldRel.y(), floorAndModuleDiffFieldRel.x())}};

    btVector3 speedsDiffFrictionForce{forceMag.value() * forceDir.Cos(), forceMag.value() * forceDir.Sin(), 0};

    frc::ChassisSpeeds moduleSpeedsFieldRelative = frc::ChassisSpeeds::FromRobotRelativeSpeeds(moduleSpeeds, robotRotation);
    Eigen::Vector2d currentModuleVelFieldRel{moduleSpeedsFieldRelative.vx.value(), moduleSpeedsFieldRelative.vy.value()};

    frc::Rotation2d dTheta{units::radian_t{std::atan2(currentModuleVelFieldRel.y(), currentModuleVelFieldRel.x()) -
                                           std::atan2(previousModuleSpeedsFieldRelative.y(), previousModuleSpeedsFieldRelative.x())}};

    units::radians_per_second_t orbitalAngularVel = dTheta.Radians() / SimulatedArena::GetSimulationDt();
    frc::Rotation2d centripetalDir =
        units::radian_t{std::atan2(previousModuleSpeedsFieldRelative.y(), previousModuleSpeedsFieldRelative.x())} + 90_deg;

    units::newton_t centripetalMag =
        units::meters_per_second_t{previousModuleSpeedsFieldRelative.norm()} * orbitalAngularVel * config.robotMass / 1_rad;
    btVector3 centripetalForce{centripetalMag.value() * centripetalDir.Cos(), centripetalMag.value() * centripetalDir.Sin(), 0};

    previousModuleSpeedsFieldRelative = currentModuleVelFieldRel;

    btVector3 totalFrictionForce = speedsDiffFrictionForce + centripetalForce;
    if (totalFrictionForce.length() > totalGrippingForce.value())
        totalFrictionForce = totalFrictionForce.normalize() * totalGrippingForce.value();

    body->applyCentralForce(totalFrictionForce);
}

void SwerveDriveSimulation::SimulateChassisFrictionTorque() {
    units::radians_per_second_t maxAngVel = MaxAngularVelocity();
    units::radians_per_second_t currentAngVel{body->getAngularVelocity().z()};
    units::radians_per_second_t desiredAngVel = GetDesiredSpeeds().omega;

    double actualRotPercent = units::math::abs(currentAngVel / maxAngVel);
    double desiredRotPercent = units::math::abs(desiredAngVel / maxAngVel);

    if (actualRotPercent < 0.01 && desiredRotPercent < 0.02)
        body->setAngularVelocity({0, 0, 0});
    else {
        units::radians_per_second_t diff = GetModuleSpeeds().omega - currentAngVel;
        units::newton_t grippingForce = moduleSimulations[0].config.GetGrippingForce(gravityForceOnEachModule);
        units::newton_meter_t grippingTorque = grippingForce * moduleTranslations[0].Norm() * moduleSimulations.size();

        double FRICTION_TORQUE_GAIN = 1;
        units::newton_meter_t appliedTorque =
            units::math::min(FRICTION_TORQUE_GAIN * grippingTorque * units::math::abs(diff).value(), grippingTorque);

        body->applyTorque({0, 0, units::math::copysign(appliedTorque, diff).value()});
    }
}

void SwerveDriveSimulation::SimulateModulePropellingForces() {
    btTransform worldTrans = body->getWorldTransform();

    for (int i = 0; i < moduleSimulations.size(); i++) {
        btVector3 localPos{moduleTranslations[i].X().value(), moduleTranslations[i].Y().value(), 0};
        btVector3 worldPos = worldTrans * localPos;

        btVector3 velocityAtModule = body->getVelocityInLocalPoint(localPos);

        Eigen::Vector2d moduleForce = moduleSimulations[i].UpdateSimulationSubTickGetModuleForce(
            {velocityAtModule.x(), velocityAtModule.y()}, GetSimulatedDriveTrainPose().Rotation(), gravityForceOnEachModule);

        body->applyForce({moduleForce.x(), moduleForce.y(), 0}, worldPos);
    }
}

frc::ChassisSpeeds SwerveDriveSimulation::GetDesiredSpeeds() const {
    std::vector<frc::SwerveModuleState> states;
    for (const auto& sim : moduleSimulations)
        states.push_back(sim.GetFreeSpinState());
    return kinematics.ToChassisSpeeds(states);
}

frc::ChassisSpeeds SwerveDriveSimulation::GetModuleSpeeds() const {
    std::vector<frc::SwerveModuleState> states;
    for (const auto& sim : moduleSimulations)
        states.push_back(sim.GetCurrentState());
    return kinematics.ToChassisSpeeds(states);
}
