#include "maplesim/motorsims/MapleMotorSim.h"

using namespace maplesim;

void MapleMotorSim::Update(units::second_t dt) {
    if (controller)
        appliedVoltage = controller->UpdateControlSignal(state.mechanismAngularPosition, state.mechanismAngularVelocity,
                                                         state.mechanismAngularPosition * configs.gearing,
                                                         state.mechanismAngularVelocity * configs.gearing);
    else
        appliedVoltage = 0_V;
    appliedVoltage = SimulatedBattery::Clamp(appliedVoltage);
    statorCurrent = configs.CalculateCurrent(state.mechanismAngularVelocity, appliedVoltage);
    state.Step(configs.CalculateTorque(statorCurrent), configs.friction, configs.loadMOI, dt);

    if (state.mechanismAngularPosition < configs.reverseHardwareLimit)
        state = {configs.reverseHardwareLimit, 0_rad_per_s};
    else if (state.mechanismAngularPosition > configs.forwardHardwareLimit)
        state = {configs.forwardHardwareLimit, 0_rad_per_s};
}
