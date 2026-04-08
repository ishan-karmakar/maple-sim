#pragma once
#include <units/voltage.h>
#include <units/current.h>
#include <frc/filter/LinearFilter.h>
#include <frc/MathUtil.h>

namespace maplesim {

class MapleMotorSim;

class SimulatedBattery {
   public:
    static inline void AddElectricalAppliances(std::function<units::ampere_t(void)> customElectricalAppliances) {
        electricalAppliances.push_back(customElectricalAppliances);
    }

    static void AddMotor(MapleMotorSim& mapleMotorSim);

    static void SimulationSubTick();

    static inline units::volt_t GetBatteryVoltage() { return batteryVoltage; }

    static units::ampere_t GetTotalCurrentDrawn();

    static constexpr units::volt_t Clamp(units::volt_t voltage) { return std::clamp(voltage, -batteryVoltage, batteryVoltage); }

   private:
    static constexpr units::volt_t BATTERY_NOMINAL_VOLTAGE = 13.5_V;
    static frc::LinearFilter<units::ampere_t> currentFilter;
    static std::vector<std::function<units::ampere_t(void)>> electricalAppliances;
    static units::volt_t batteryVoltage;
};

}  // namespace maplesim
