// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.Planners;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.uni.frc.Constants;
import com.uni.frc.subsystems.RobotState;
import com.uni.frc.subsystems.Vision.OdometryLimeLight.VisionUpdate;
import com.uni.lib.HeadingController;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Translation2d;
import com.uni.lib.geometry.Twist2d;
import com.uni.lib.util.InterpolatingDouble;
import com.uni.lib.util.InterpolatingTreeMap;

/** Add your docs here. */
public class AimingPlanner {

    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mShotTimeMap = Constants.ShooterConstants.SHOT_TRAVEL_TIME_TREE_MAP;

    private Pose2d mFieldToSpeaker;
    private AimingRequest mAimingRequest;
    private boolean isAimed = false;

    public enum AimingRequest {
        SPEAKER,
        LOB,
        PIECE,
    }

    public AimingPlanner() {
    }

    public AimingRequest getAimingRequest() {
        return mAimingRequest;
    }

    public double updateAiming(double timeStamp, Pose2d currentOdomToRobot, Pose2d visionPoseComponent,
            AimingRequest request, Pose2d notePose, HeadingController headingController,
            Twist2d currentVelocity) {
        Pose2d targetPose = new Pose2d();
        mAimingRequest = request;
        switch (mAimingRequest) {
            case SPEAKER:
                mFieldToSpeaker = Constants.getSpeakerAimingPose();
                break;
            case LOB:
                mFieldToSpeaker = Constants.getLobPose();
                break;
            case PIECE:
                mFieldToSpeaker = notePose;
                break;
        }
        Logger.recordOutput("aimPose", mAimingRequest);
        double estimatedTimeFrame = 0;
        Pose2d odomToTargetPoint = visionPoseComponent.inverse().transformBy(mFieldToSpeaker);
        double travelDistance = odomToTargetPoint.transformBy(currentOdomToRobot).getTranslation().norm();
        estimatedTimeFrame = mShotTimeMap.getInterpolated(new InterpolatingDouble(travelDistance)).value;
        Pose2d poseAtTimeFrame = RobotState.getInstance().getPredictedPoseFromOdometry(estimatedTimeFrame);

        Logger.recordOutput("poseAttmie", poseAtTimeFrame.toWPI());
        Pose2d futureOdomToTargetPoint = poseAtTimeFrame.inverse().transformBy(odomToTargetPoint).inverse();
        Rotation2d targetRotation = futureOdomToTargetPoint.getTranslation().getAngle().inverse();
        targetPose = new Pose2d(futureOdomToTargetPoint.getTranslation(), targetRotation);

        switch (mAimingRequest) {
            case SPEAKER:
                headingController.setTargetHeading(targetPose.getRotation().inverse());
                
                break;
        
            case LOB:
                headingController.setTargetHeading(targetPose.getRotation().inverse());
                break;
            
            case PIECE:
                if (notePose.getTranslation().distance(Translation2d.identity()) == 0) {
                    headingController.setTargetHeading(currentOdomToRobot.getRotation().inverse());
                    break;
                }
                headingController.setTargetHeading(targetPose.getRotation().inverse().flip());
                break;
        }
        double rotationOutput = headingController.updateRotationCorrection(currentOdomToRobot.getRotation().inverse().rotateBy(-4),
                timeStamp);
        Logger.recordOutput("aimingoutput", rotationOutput);
        isAimed = headingController.atTarget();
        return rotationOutput;

    }

    public boolean isAimed() {
        return isAimed;
    }

}
