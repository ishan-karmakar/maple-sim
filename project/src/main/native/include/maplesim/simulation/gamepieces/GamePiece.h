#pragma once
#include <frc/geometry/Pose3d.h>

namespace maplesim {

class GamePiece {
   public:
    virtual frc::Pose3d GetPose3d() = 0;

    virtual std::string GetType() = 0;

    virtual frc::Translation3d GetVelocity3dMPS() = 0;

    virtual void TriggerHitTargetCallback() = 0;

    virtual bool IsGrounded() = 0;
};

}  // namespace maplesim
