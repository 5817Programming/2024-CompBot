// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.Planners;

import java.util.Optional;
import java.util.OptionalDouble;


import com.wcp.frc.Constants;
import com.wcp.frc.subsystems.RobotState;
import com.wcp.frc.subsystems.Vision.LimeLight.VisionUpdate;
import com.wcp.lib.HeadingController;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;

import com.wcp.lib.util.InterpolatingDouble;
import com.wcp.lib.util.InterpolatingTreeMap;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class AimingUtils {

    private boolean mAimingComplete = false;
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mShotTimeMap = Constants.ShooterConstants.SHOT_TRAVEL_TIME_TREE_MAP;

    private Pose2d mFieldToSpeaker;
    private OptionalDouble mStartingTime;
    private Optional<VisionUpdate> mVisionUpdate;
    private AimingRequest mAimingRequest;

    public enum AimingRequest{
        Odometry,
        LimeLight,
    }

    public void reset(){
        mAimingComplete = false;
        mStartingTime = OptionalDouble.of(Timer.getFPGATimestamp());
    }

    public AimingRequest getAimingRequest(){
        return mAimingRequest;
    }

    public Optional<Pose2d> updateAiming(double timeStamp, Pose2d currentOdomToRobot, Pose2d currentFieldToOdometry, AimingRequest request,VisionUpdate visionUpdate, HeadingController headingController){
        Optional<Pose2d> targetPose = Optional.empty();
        mAimingRequest = request;
        mVisionUpdate = Optional.ofNullable(visionUpdate);
        if(mAimingRequest == AimingRequest.LimeLight && mVisionUpdate.isEmpty()){
            System.out.println("No Vision Update For Aiming, Switching To Odometry");
            mAimingRequest = AimingRequest.Odometry;
        }
        switch (mAimingRequest) {
            case Odometry:
                Pose2d odomToTargetPoint = currentFieldToOdometry.inverse().transformBy(mFieldToSpeaker);
                double travelDistance = odomToTargetPoint.getTranslation().norm();
                InterpolatingDouble estimatedTimeFrame = mShotTimeMap.getInterpolated(new InterpolatingDouble(travelDistance));
                Pose2d poseAtTimeFrame = RobotState.getInstance().getPredictedPoseFromOdometry(estimatedTimeFrame.value);
                Pose2d futureOdomToTargetPoint = poseAtTimeFrame.inverse().transformBy(mFieldToSpeaker);
                Rotation2d targetRotation = futureOdomToTargetPoint.getTranslation().getAngle();
                targetPose = Optional.of(new Pose2d(futureOdomToTargetPoint.getTranslation(), targetRotation));
            case LimeLight:
                if(mVisionUpdate.isPresent()){
                Pose2d cameraTotag = Pose2d.fromTranslation(mVisionUpdate.get().getCameraToTarget().rotateBy(Constants.VisionConstants.cameraYawOffset));
                Pose2d tagToRobot = Pose2d.fromTranslation(Constants.VisionConstants.ROBOT_TO_CAMERA.transformBy(cameraTotag).getTranslation().rotateBy(currentOdomToRobot.getRotation()));
                targetPose = Optional.of(new Pose2d(tagToRobot.getTranslation(),tagToRobot.getTranslation().getAngle()));
                } else{
                    System.out.println("How did you get here");
                }
        }

        if(targetPose.isEmpty()) return targetPose;
        headingController.setTargetHeading(targetPose.get().getRotation());

        mAimingComplete = headingController.atTarget();
        double rotationOutput = headingController.updateRotationCorrection(currentOdomToRobot.getRotation(), timeStamp);

        targetPose = Optional.of(new Pose2d(
            targetPose.get().getTranslation(),
            Rotation2d.fromDegrees(mAimingComplete ? 0 : rotationOutput)
            ));
        
        if(mStartingTime.isPresent() && mAimingComplete){
            System.out.println("Aiming Complete in " + (timeStamp - mStartingTime.getAsDouble()));
            mStartingTime = OptionalDouble.empty();
        }

        return targetPose;
    }

    public boolean getAimingComplete(){
        return mAimingComplete;
    }


}
