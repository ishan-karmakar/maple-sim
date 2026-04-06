#pragma once
#include <frc/system/plant/DCMotor.h>
#include <units/moment_of_inertia.h>

namespace maplesim {

class SimMotorConfigs {
    public:
    constexpr SimMotorConfigs(frc::DCMotor motor, double gearing, units::kilogram_square_meter_t loadMOI, units::volt_t frictionVoltage) : motor{motor}, gearing{gearing}, loadMOI{loadMOI}, friction{motor.Torque(motor.Current(0_rad_per_s, frictionVoltage))} {}

    constexpr units::volt_t CalculateVoltage(units::ampere_t current, units::radians_per_second_t mechanismVelocity) const {
        return motor.Voltage(motor.Torque(current), mechanismVelocity * gearing);
    }

    constexpr units::radians_per_second_t CalculateMechanismVelocity(units::ampere_t current, units::volt_t voltage) const {
        return motor.Speed(motor.Torque(current), voltage) / gearing;
    }

    constexpr units::ampere_t CalculateCurrent(units::radians_per_second_t mechanismVelocity, units::volt_t voltage) const {
        return motor.Current(mechanismVelocity * gearing, voltage);
    }

    constexpr units::ampere_t CalculateCurrent(units::newton_meter_t torque) const {
        return motor.Current(torque / gearing);
    }

    constexpr units::newton_meter_t CalculateTorque(units::ampere_t current) const {
        return motor.Torque(current) * gearing;
    }

    constexpr SimMotorConfigs& WithHardLimits(units::radian_t forwardLimit, units::radian_t reverseLimit) {
        forwardHardwareLimit = forwardLimit;
        reverseHardwareLimit = reverseLimit;
        return *this;
    }

    constexpr units::radians_per_second_t FreeSpinMechanismVelocity() const {
        return motor.freeSpeed / gearing;
    }

    constexpr units::ampere_t FreeSpinCurrent() const {
        return motor.freeCurrent;
    }

    constexpr units::ampere_t StallCurrent() const {
        return motor.stallCurrent;
    }

    constexpr units::newton_meter_t StallTorque() const {
        return motor.stallTorque;
    }

    constexpr units::volt_t NominalVoltage() const {
        return motor.nominalVoltage;
    }

    frc::DCMotor motor;
    double gearing;
    units::kilogram_square_meter_t loadMOI;
    units::newton_meter_t friction;

    protected:
    units::radian_t forwardHardwareLimit{std::numeric_limits<double>::infinity()};
    units::radian_t reverseHardwareLimit{-std::numeric_limits<double>::infinity()};
};

}