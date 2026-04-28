#pragma once
#include <units/voltage.h>
#include <units/current.h>
#include <frc/filter/LinearFilter.h>
#include <frc/MathUtil.h>

namespace maplesim {

class MapleMotorSim;

/**
 *
 *
 * <h1>Simulates the main battery of the robot.</h1>
 *
 * <p>This class simulates the behavior of a robot's battery. Electrical appliances can be added to the battery to draw
 * current. The battery voltage is affected by the current drawn from various appliances.
 */
class SimulatedBattery {
   public:
    /**
     *
     *
     * <h2>Adds a custom electrical appliance.</h2>
     *
     * <p>Connects the electrical appliance to the battery, allowing it to draw current from the battery.
     *
     * @param customElectricalAppliances The supplier for the current drawn by the appliance.
     */
    static inline void AddElectricalAppliances(std::function<units::ampere_t(void)> customElectricalAppliances) {
        electricalAppliances.push_back(customElectricalAppliances);
    }

    /**
     *
     *
     * <h2>Adds a motor to the list of electrical appliances.</h2>
     *
     * <p>The motor will draw current from the battery.
     *
     * @param mapleMotorSim The motor simulation object.
     */
    static void AddMotor(MapleMotorSim& mapleMotorSim);

    /**
     *
     *
     * <h2>Updates the battery simulation.</h2>
     *
     * <p>Calculates the battery voltage based on the current drawn by all appliances.
     *
     * <p>The battery voltage is clamped to avoid going below the brownout voltage.
     */
    static void SimulationSubTick();

    /**
     *
     *
     * <h2>Obtains the voltage of the battery.</h2>
     *
     * @return The battery voltage as a {@link Voltage} object.
     */
    static inline units::volt_t GetBatteryVoltage() { return batteryVoltage; }

    /**
     *
     *
     * <h2>Obtains the total current drawn from the battery.</h2>
     *
     * <p>Iterates through all the appliances to obtain the total current used.
     *
     * @return The total current as a {@link Current} object.
     */
    static units::ampere_t GetTotalCurrentDrawn();

    /**
     *
     *
     * <h2>Clamps the voltage according to the supplied voltage and the battery's capabilities.</h2>
     *
     * <p>If the supplied voltage exceeds the battery's maximum voltage, it will be reduced to match the battery's
     * voltage.
     *
     * @param voltage The voltage to be clamped.
     * @return The clamped voltage as a {@link Voltage} object.
     */
    static constexpr units::volt_t Clamp(units::volt_t voltage) { return std::clamp(voltage, -batteryVoltage, batteryVoltage); }

   private:
    // Nominal voltage for a fully charged battery (13.5 volts).
    static constexpr units::volt_t BATTERY_NOMINAL_VOLTAGE = 13.5_V;
    // Filter to smooth the current readings.
    static frc::LinearFilter<units::ampere_t> currentFilter;
    static std::vector<std::function<units::ampere_t(void)>> electricalAppliances;
    // The current battery voltage in volts.
    static units::volt_t batteryVoltage;
};

}  // namespace maplesim
