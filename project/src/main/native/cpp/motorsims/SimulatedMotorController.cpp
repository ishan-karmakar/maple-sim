#include <units/math.h>
#include "maplesim/motorsims/SimulatedMotorController.h"

using namespace maplesim;

units::volt_t GenericMotorController::ConstrainOutputVoltage(units::radian_t encoderAngle, units::radians_per_second_t encoderVelocity,
                                                             units::volt_t requestedVoltage) const {
    units::ampere_t kCurrentThreshold = 1.2_A;

    units::volt_t limitedVoltage = requestedVoltage;
    units::ampere_t currentAtRequestedVoltage = model.Current(encoderVelocity, requestedVoltage);
    bool currentTooHigh = units::math::abs(currentAtRequestedVoltage) > (kCurrentThreshold * currentLimit);
    if (currentTooHigh) {
        units::ampere_t limitedCurrent = units::math::copysign(currentLimit, currentAtRequestedVoltage);
        limitedVoltage = model.Voltage(model.Torque(limitedCurrent), encoderVelocity);
    }

    if (units::math::abs(limitedVoltage) > units::math::abs(requestedVoltage))
        limitedVoltage = requestedVoltage;

    if (encoderAngle > forwardSoftwareLimit && limitedVoltage > 0_V)
        limitedVoltage = 0_V;
    if (encoderAngle < reverseSoftwareLimit && limitedVoltage < 0_V)
        limitedVoltage = 0_V;

    return limitedVoltage;
}
