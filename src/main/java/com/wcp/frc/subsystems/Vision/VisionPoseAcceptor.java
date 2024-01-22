package com.wcp.frc.subsystems.Vision;

import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Twist2d;

import edu.wpi.first.wpilibj.DriverStation;


public class VisionPoseAcceptor {
    public boolean shouldAcceptVision(double timestamp, Pose2d visionFieldToVehicle, Pose2d vehicleToTag, Twist2d robotVelocity) {
        if (DriverStation.isAutonomous()) {
            double kMaxRange = 4.0;
            if (vehicleToTag.getTranslation().norm() > kMaxRange) {
                return false;
            }
        }

        boolean rotatingTooFast = Math.abs(robotVelocity.dtheta) >= 1.0;
        if (!rotatingTooFast) {
            return true;
        } else {
            return false;
        }
    }

}