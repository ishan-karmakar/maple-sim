#pragma once
#include <string>
#include <frc/Errors.h>

namespace maplesim {

class BoundingCheck {
   public:
    template <typename T>
    static inline void Check(T value, T lowerBound, T upperBound, std::string variableName, std::string unit) {
        if (lowerBound <= value && value <= upperBound)
            return;
        FRC_ReportError(frc::err::InvalidParameter, "The provided \"{}\" is {}{} which seems abnormal, please check its correctness", value,
                        unit);
    }
};

}  // namespace maplesim
