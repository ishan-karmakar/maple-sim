#pragma once
#include "maplesim/motorsims/SimMotorConfigs.h"
#include "maplesim/motorsims/SimMotorState.h"
#include "maplesim/motorsims/SimulatedMotorController.h"
#include "maplesim/motorsims/SimulatedBattery.h"

namespace maplesim {

/**
 *
 *
 * <h1>{@link edu.wpi.first.wpilibj.simulation.DCMotorSim} with a bit of extra spice.</h1>
 *
 * <p>This class extends the functionality of the original {@link edu.wpi.first.wpilibj.simulation.DCMotorSim} and
 * models the following aspects in addition:
 *
 * <ul>
 *   <li>Motor Controller Closed Loops.
 *   <li>Smart current limiting.
 *   <li>Friction force on the rotor.
 * </ul>
 */
class MapleMotorSim {
   public:
    /**
     *
     *
     * <h2>Constructs a Brushless Motor Simulation Instance.</h2>
     *
     * @param configs the configuration for this motor
     */
    inline MapleMotorSim(SimMotorConfigs configs) : configs{configs} { SimulatedBattery::AddMotor(*this); }

    /**
     *
     *
     * <h2>Updates the simulation.</h2>
     *
     * <p>This is equivalent to{@link edu.wpi.first.wpilibj.simulation.DCMotorSim#update(double)}.
     */
    void Update(units::second_t dt);

    template <typename T>
    inline T& UseMotorController(std::unique_ptr<T> motorController) {
        controller = std::move(motorController);
        return static_cast<T&>(*controller);
    }

    inline GenericMotorController& UseSimpleDCMotorController() {
        return UseMotorController(std::make_unique<GenericMotorController>(configs.motor));
    }

    /**
     *
     *
     * <h2>Obtains the <strong>final</strong> position of the mechanism.</h2>
     *
     * <p>This is equivalent to {@link edu.wpi.first.wpilibj.simulation.DCMotorSim#getAngularPosition()}.
     *
     * @return the angular position of the mechanism, continuous
     */
    constexpr units::radian_t GetAngularPosition() const { return state.mechanismAngularPosition; }

    /**
     *
     *
     * <h2>Obtains the angular position measured by the relative encoder of the motor.</h2>
     *
     * @return the angular position measured by the encoder, continuous
     */
    constexpr units::radian_t GetEncoderPosition() const { return GetAngularPosition() * configs.gearing; }

    /**
     *
     *
     * <h2>Obtains the <strong>final</strong> velocity of the mechanism.</h2>
     *
     * <p>This is equivalent to {@link edu.wpi.first.wpilibj.simulation.DCMotorSim#getAngularVelocity()}.
     *
     * @return the final angular velocity of the mechanism
     */
    constexpr units::radians_per_second_t GetVelocity() const { return state.mechanismAngularVelocity; }

    /**
     *
     *
     * <h2>Obtains the angular velocity measured by the relative encoder of the motor.</h2>
     *
     * @return the angular velocity measured by the encoder
     */
    constexpr units::radians_per_second_t GetEncoderVelocity() const { return GetVelocity() * configs.gearing; }

    /**
     *
     *
     * <h2>Obtains the applied voltage by the motor controller.</h2>
     *
     * <p>The applied voltage is calculated by the motor controller in the previous call to {@link #update(Time)}
     *
     * <p>The motor controller specified by {@link #useMotorController(SimulatedMotorController)} is used to calculate
     * the applied voltage.
     *
     * <p>The applied voltage is also restricted for current limit and battery voltage.
     *
     * @return the applied voltage
     */
    constexpr units::volt_t GetAppliedVoltage() const { return appliedVoltage; }

    /**
     *
     *
     * <h2>Obtains the <strong>stator</strong> current.</h2>
     *
     * <p>This is equivalent to {@link DCMotorSim#getCurrentDrawAmps()}
     *
     * @return the stator current of the motor
     */
    constexpr units::ampere_t GetStatorCurrent() const { return statorCurrent; }

    /**
     *
     *
     * <h2>Obtains the <strong>supply</strong> current.</h2>
     *
     * <p>The supply current is different from the stator current, as described <a
     * href='https://www.chiefdelphi.com/t/current-limiting-talonfx-values/374780/10'>here</a>.
     *
     * @return the supply current of the motor
     */
    inline units::ampere_t GetSupplyCurrent() const { return GetStatorCurrent() * appliedVoltage / SimulatedBattery::GetBatteryVoltage(); }

    /**
     *
     *
     * <h2>Obtains the configuration of the motor.</h2>
     *
     * <p>You can modify the configuration of this motor by:
     *
     * <pre><code>
     *     mapleMotorSim.getConfigs()
     *          .with...(...)
     *          .with...(...);
     * </code></pre>
     *
     * @return the configuration of the motor
     */
    constexpr SimMotorConfigs GetConfigs() const { return configs; }

   private:
    SimMotorConfigs configs;

    SimMotorState state{0_rad, 0_rad_per_s};
    std::unique_ptr<SimulatedMotorController> controller;
    units::volt_t appliedVoltage;
    units::ampere_t statorCurrent;
};

}  // namespace maplesim
