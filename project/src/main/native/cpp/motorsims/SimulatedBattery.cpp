#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/Errors.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "maplesim/motorsims/SimulatedBattery.h"

using namespace maplesim;

frc::LinearFilter<units::ampere_t> SimulatedBattery::currentFilter = frc::LinearFilter<units::ampere_t>::MovingAverage(50);
std::vector<std::function<units::ampere_t(void)>> SimulatedBattery::electricalAppliances;
units::volt_t SimulatedBattery::batteryVoltage = SimulatedBattery::BATTERY_NOMINAL_VOLTAGE;

void SimulatedBattery::SimulationSubTick() {
    units::ampere_t totalCurrent = currentFilter.Calculate(GetTotalCurrentDrawn());

    batteryVoltage = frc::sim::BatterySim::Calculate(BATTERY_NOMINAL_VOLTAGE, 20_mOhm, {totalCurrent});

    if (batteryVoltage < frc::sim::RoboRioSim::GetBrownoutVoltage()) {
        batteryVoltage = frc::sim::RoboRioSim::GetBrownoutVoltage();
        FRC_ReportError(frc::err::Error, "[MapleSim] BrownOut Detected, protecting battery voltage...");
    }

    frc::sim::RoboRioSim::SetVInVoltage(batteryVoltage);

    frc::SmartDashboard::PutNumber("BatterySim/TotalCurrent (Amps)", totalCurrent.value());
    frc::SmartDashboard::PutNumber("BatterySim/BatteryVoltage (Volts)", batteryVoltage.value());
}
