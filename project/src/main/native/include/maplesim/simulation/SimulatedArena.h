#pragma once
#include <unordered_map>
#include <networktables/DoubleTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/Timer.h>
#include <frc/DriverStation.h>

namespace maplesim {

class SimulatedArena {
   public:
    class Simulatable {
       public:
        virtual void SimulationSubTick(int subTickNum) = 0;
    };

    static SimulatedArena& GetInstance();

    static inline void OverrideInstance(SimulatedArena&& newInstance) { GetInstance() = std::move(newInstance); }

    static inline int GetSimulationSubTicksIn1Period() { return SIMULATION_SUB_TICKS_IN_1_PERIOD; }

    static inline units::second_t GetSimulationDt() { return SIMULATION_DT; }

    constexpr int GetScore(bool isBlue) const { return isBlue ? blueScore : redScore; }

    constexpr int GetScore(frc::DriverStation::Alliance allianceColor) const {
        return GetScore(allianceColor == frc::DriverStation::kBlue);
    }

    void AddToScore(bool isBlue, int toAdd);

    static inline void OverrideSimulationTimings(units::second_t robotPeriod, int simulationSubTicksPerPeriod) {
        SIMULATION_SUB_TICKS_IN_1_PERIOD = simulationSubTicksPerPeriod;
        SIMULATION_DT = robotPeriod / SIMULATION_SUB_TICKS_IN_1_PERIOD;
    }

    inline void AddCustomSimulation(std::unique_ptr<Simulatable> simulatable) { customSimulations.push_back(std::move(simulatable)); }

    constexpr void EnableBreakdownPublishing() { shouldPublishMatchBreakdown = true; }

    constexpr void DisableBreakdownPublishing() { shouldPublishMatchBreakdown = false; }

    void ReplaceValueInMatchBreakdown(bool isBlueTeam, std::string valueKey, double value);

    inline void SetupValueForMatchBreakdown(std::string valueKey) {
        ReplaceValueInMatchBreakdown(true, valueKey, 0);
        ReplaceValueInMatchBreakdown(false, valueKey, 0);
    }

    void AddValueToMatchBreakdown(bool isBlueTeam, std::string valueKey, double toAdd);

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
    void PublishBreakdown();

    int redScore = 0;
    int blueScore = 0;
    frc::Timer matchClock;

    std::unordered_map<std::string, nt::DoublePublisher> redPublishers;
    std::unordered_map<std::string, nt::DoublePublisher> bluePublishers;

    std::vector<std::unique_ptr<Simulatable>> customSimulations;

   private:
    static int SIMULATION_SUB_TICKS_IN_1_PERIOD;
    static units::second_t SIMULATION_DT;

    bool shouldPublishMatchBreakdown = true;
};

}  // namespace maplesim
