// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.Planners;

import java.util.Optional;
import java.util.OptionalDouble;

import javax.swing.text.html.Option;

import com.wcp.frc.Constants;
import com.wcp.frc.CompConstants.VisionConstants;
import com.wcp.frc.subsystems.RobotState;
import com.wcp.frc.subsystems.Vision.LimeLight.VisionUpdate;
import com.wcp.lib.HeadingController;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.geometry.Twist2d;
import com.wcp.lib.util.InterpolatingDouble;
import com.wcp.lib.util.InterpolatingTreeMap;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class AimingUtils {

    private boolean mAimingComplete = false;
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mShotTimeMap = Constants.ShooterConstants.SHOT_TRAVEL_TIME_TREE_MAP;

    private Pose2d mFieldToSpeaker;
    private OptionalDouble mStartingTime;

    public enum AimingRequest{
        Odometry,
        LimeLight,
        MANUAL
    }

    public void reset(){
        mAimingComplete = false;
        mStartingTime = OptionalDouble.of(Timer.getFPGATimestamp());
    }

    public Optional<Pose2d> updateAiming(double timeStamp, Pose2d currentOdomToRobot, Pose2d currentFieldToOdometry, AimingRequest request,VisionUpdate visionUpdate){
        Optional<Pose2d> targetPose = Optional.empty();

        switch (request) {
            case Odometry:
                Pose2d odomToTargetPoint = currentFieldToOdometry.inverse().transformBy(mFieldToSpeaker);
                double travelDistance = odomToTargetPoint.getTranslation().norm();
                InterpolatingDouble estimatedTimeFrame = mShotTimeMap.getInterpolated(new InterpolatingDouble(travelDistance));
                Pose2d poseAtTimeFrame = RobotState.getInstance().getPredictedPoseFromOdometry(estimatedTimeFrame.value);
                Pose2d futureOdomToTargetPoint = poseAtTimeFrame.inverse().transformBy(mFieldToSpeaker);
                Rotation2d targetRotation = futureOdomToTargetPoint.getTranslation().getAngle();
            case LimeLight:
                Translation2d cameraTotag = visionUpdate.getCameraToTarget();
                Pose2d tagToRobot = Constants.VisionConstants.ROBOT_TO_CAMERA.
                 targetPose = Optional.of(new Pose2d(,visionUpdate.getCameraToTarget().getAngle()));
            case MANUAL:

                
        }
  
    }


}
