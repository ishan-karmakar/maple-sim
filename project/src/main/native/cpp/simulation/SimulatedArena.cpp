#include <frc/RobotBase.h>
#include "maplesim/simulation/SimulatedArena.h"

using namespace maplesim;

bool SimulatedArena::ALLOW_CREATION_ON_REAL_ROBOT = false;

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
