#include <frc/RobotBase.h>
#include <frc/TimedRobot.h>
#include <btBulletDynamicsCommon.h>
#include "maplesim/simulation/SimulatedArena.h"

using namespace maplesim;

bool SimulatedArena::ALLOW_CREATION_ON_REAL_ROBOT = false;
int SimulatedArena::SIMULATION_SUB_TICKS_IN_1_PERIOD = 5;
units::second_t SimulatedArena::SIMULATION_DT = frc::TimedRobot::kDefaultPeriod / SimulatedArena::SIMULATION_SUB_TICKS_IN_1_PERIOD;

nt::BooleanPublisher SimulatedArena::resetFieldPublisher =
    nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard/MapleSim/MatchData")->GetBooleanTopic("Reset Field").Publish();
nt::BooleanSubscriber SimulatedArena::resetFieldSubscriber = resetFieldPublisher.GetTopic().Subscribe(false);

SimulatedArena& SimulatedArena::GetInstance() {
    if (frc::RobotBase::IsReal() && !ALLOW_CREATION_ON_REAL_ROBOT)
        throw std::runtime_error{
            "MapleSim is running on a real robot! (If you would actually want that, set SimulatedArena::ALLOW_CREATION_ON_REAL_ROBOT to "
            "true)."};
    static SimulatedArena instance;
    return instance;
}

SimulatedArena::SimulatedArena(FieldMap obstaclesMap) {
    collisionConfiguration = std::make_unique<btDefaultCollisionConfiguration>();
    collisionDispatcher = std::make_unique<btCollisionDispatcher>();
    broadphase = std::make_unique<btDbvtBroadphase>();
    constraintSolver = std::make_unique<btSequentialImpulseConstraintSolver>();
    physicsWorld = std::make_unique<btDiscreteDynamicsWorld>(collisionDispatcher.get(), broadphase.get(), collisionConfiguration.get(),
                                                             collisionConfiguration.get());

    for (const auto& obstacle : obstaclesMap.obstacles)
        physicsWorld->addRigidBody(obstacle.get());

    SetupValueForMatchBreakdown("TotalScore");
    SetupValueForMatchBreakdown("TeleopScore");
    SetupValueForMatchBreakdown("Auto/AutoScore");
    resetFieldPublisher.Set(false);
    matchClock.Start();
}

void SimulatedArena::AddToScore(bool isBlue, int toAdd) {
    if (isBlue)
        blueScore += toAdd;
    else
        redScore += toAdd;
    AddValueToMatchBreakdown(isBlue, frc::DriverStation::IsAutonomous() ? "Auto/AutoScore" : "TeleopScore", toAdd);
}

void SimulatedArena::ReplaceValueInMatchBreakdown(bool isBlueTeam, std::string valueKey, double value) {
    if (isBlueTeam)
        blueScoringBreakdown[valueKey] = value;
    else
        redScoringBreakdown[valueKey] = value;
}

void SimulatedArena::AddValueToMatchBreakdown(bool isBlueTeam, std::string valueKey, double toAdd) {
    if (isBlueTeam)
        blueScoringBreakdown[valueKey] += toAdd;
    else
        redScoringBreakdown[valueKey] += toAdd;
}

void SimulatedArena::FieldMap::AddBorderLine(const frc::Translation2d& startingPoint, const frc::Translation2d& endingPoint) {
    frc::Translation2d direction = endingPoint - startingPoint;
    units::meter_t distance = direction.Norm();
    frc::Pose2d position{(startingPoint + endingPoint) / 2, direction.Angle()};
    AddCustomObstacle(new btBoxShape{{distance.value() / 2, 0.01, 0.1}}, position);
}

void SimulatedArena::FieldMap::AddRectangularObstacle(units::meter_t width, units::meter_t height,
                                                      const frc::Pose2d& absolutePositionOnField) {
    AddCustomObstacle(new btBoxShape{{width.value() / 2, height.value() / 2, 0.1}}, absolutePositionOnField);
}

void SimulatedArena::FieldMap::AddCustomObstacle(btCollisionShape* shape, const frc::Pose2d& absolutePositionOnField) {
    auto obstacle = CreateObstacle(shape);
    btTransform transform = btTransform::getIdentity();
    transform.setOrigin({absolutePositionOnField.X().value(), absolutePositionOnField.Y().value(), 0});
    units::radian_t angle = absolutePositionOnField.Rotation().Radians() / 2;
    transform.setRotation(btQuaternion{0, 0, units::math::sin(angle), units::math::cos(angle)});

    obstacle->setWorldTransform(transform);
    obstacles.push_back(obstacle);
}

SimulatedArena::FieldMap::~FieldMap() {
    for (auto& body : obstacles)
        delete body->getCollisionShape();
}

std::unique_ptr<btRigidBody> SimulatedArena::FieldMap::CreateObstacle(btCollisionShape* shape) {
    btRigidBody::btRigidBodyConstructionInfo info{0, nullptr, shape};
    auto body = std::make_unique<btRigidBody>(info);
    body->setLinearFactor({1, 1, 0});
    body->setAngularFactor({0, 0, 1});
    body->setFriction(0.6);
    body->setRestitution(0.3);
    return body;
}
