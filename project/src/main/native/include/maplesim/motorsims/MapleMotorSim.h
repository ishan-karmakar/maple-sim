#pragma once
#include "maplesim/motorsims/SimMotorConfigs.h"
#include "maplesim/motorsims/SimMotorState.h"
#include "maplesim/motorsims/SimulatedMotorController.h"
#include "maplesim/motorsims/SimulatedBattery.h"

namespace maplesim {

class MapleMotorSim {
   public:
    inline MapleMotorSim(SimMotorConfigs configs) : configs{configs} { SimulatedBattery::AddMotor(*this); }

    void Update(units::second_t dt);

    template <typename T>
    inline T& UseMotorController(std::unique_ptr<T> motorController) {
        controller = std::move(motorController);
        return static_cast<T&>(*controller);
    }

    inline GenericMotorController& UseSimpleDCMotorController() {
        return UseMotorController(std::make_unique<GenericMotorController>(configs.motor));
    }

    constexpr units::radian_t GetAngularPosition() const { return state.mechanismAngularPosition; }

    constexpr units::radian_t GetEncoderPosition() const { return GetAngularPosition() * configs.gearing; }

    constexpr units::radians_per_second_t GetVelocity() const { return state.mechanismAngularVelocity; }

    constexpr units::radians_per_second_t GetEncoderVelocity() const { return GetVelocity() * configs.gearing; }

    constexpr units::volt_t GetAppliedVoltage() const { return appliedVoltage; }

    constexpr units::ampere_t GetStatorCurrent() const { return statorCurrent; }

    inline units::ampere_t GetSupplyCurrent() const { return GetStatorCurrent() * appliedVoltage / SimulatedBattery::GetBatteryVoltage(); }

    constexpr SimMotorConfigs GetConfigs() const { return configs; }

   private:
    SimMotorConfigs configs;

    SimMotorState state{0_rad, 0_rad_per_s};
    std::unique_ptr<SimulatedMotorController> controller;
    units::volt_t appliedVoltage;
    units::ampere_t statorCurrent;
};

}  // namespace maplesim
