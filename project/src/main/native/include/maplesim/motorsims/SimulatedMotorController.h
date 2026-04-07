#pragma once
#include <frc/system/plant/DCMotor.h>

namespace maplesim {

class SimulatedMotorController {
   public:
    virtual units::volt_t UpdateControlSignal(units::radian_t mechanismAngle, units::radians_per_second_t mechanismVelocity,
                                              units::radian_t encoderAngle, units::radians_per_second_t encoderVelocity) = 0;
};

class GenericMotorController : public SimulatedMotorController {
   public:
    constexpr GenericMotorController(frc::DCMotor model) : model{model} {}

    constexpr GenericMotorController& WithCurrentLimit(units::ampere_t currentLimit) {
        this->currentLimit = currentLimit;
        return *this;
    }

    constexpr GenericMotorController& WithSoftwareLimits(units::radian_t forwardSoftwareLimit, units::radian_t reverseSoftwareLimit) {
        this->forwardSoftwareLimit = forwardSoftwareLimit;
        this->reverseSoftwareLimit = reverseSoftwareLimit;
        return *this;
    }

    constexpr void RequestVoltage(units::volt_t voltage) { requestedVoltage = voltage; }

    units::volt_t ConstrainOutputVoltage(units::radian_t encoderAngle, units::radians_per_second_t encoderVelocity,
                                         units::volt_t requestedVoltage) const;

    constexpr units::volt_t UpdateControlSignal(units::radian_t mechanismAngle, units::radians_per_second_t mechanismVelocity,
                                                units::radian_t encoderAngle, units::radians_per_second_t encoderVelocity) {
        appliedVoltage = ConstrainOutputVoltage(encoderAngle, encoderVelocity, requestedVoltage);
        return appliedVoltage;
    }

    constexpr units::volt_t GetAppliedVoltage() const { return appliedVoltage; }

   private:
    frc::DCMotor model;
    units::ampere_t currentLimit = 150_A;
    units::radian_t forwardSoftwareLimit{std::numeric_limits<double>::infinity()};
    units::radian_t reverseSoftwareLimit{-std::numeric_limits<double>::infinity()};

    units::volt_t requestedVoltage;
    units::volt_t appliedVoltage;
};

}  // namespace maplesim
