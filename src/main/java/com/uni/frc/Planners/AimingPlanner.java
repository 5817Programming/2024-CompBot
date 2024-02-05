// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.Planners;

import java.util.Optional;
import com.uni.frc.Constants;
import com.uni.frc.subsystems.RobotState;
import com.uni.frc.subsystems.Vision.OdometryLimeLight.VisionUpdate;
import com.uni.lib.HeadingController;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Twist2d;
import com.uni.lib.util.InterpolatingDouble;
import com.uni.lib.util.InterpolatingTreeMap;

/** Add your docs here. */
public class AimingPlanner {

    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mShotTimeMap = Constants.ShooterConstants.SHOT_TRAVEL_TIME_TREE_MAP;

    private Pose2d mFieldToSpeaker= new Pose2d(16.28,5.45,new Rotation2d());
    private Optional<VisionUpdate> mVisionUpdate;
    private AimingRequest mAimingRequest;
    private double lastVisionTimestamp;

    public enum AimingRequest{
        Odometry,
        LimeLight,
    }

    public AimingRequest getAimingRequest(){
        return mAimingRequest;
    }

    public Pose2d updateAiming(double timeStamp, Pose2d currentOdomToRobot, Pose2d visionPoseComponent, AimingRequest request,Optional<VisionUpdate> visionUpdate, HeadingController headingController, Twist2d currentVelocity){
        Pose2d targetPose = new Pose2d();
        mAimingRequest = request;
        if(mAimingRequest == AimingRequest.LimeLight && mVisionUpdate.isEmpty()){
            System.out.println("No Vision Update For Aiming, Switching To Odometry: AimingUtils");
            mAimingRequest = AimingRequest.Odometry;
        }else{
            lastVisionTimestamp = mVisionUpdate.get().getTimestamp();
        }

        if(timeStamp - lastVisionTimestamp > 10){
            mAimingRequest = AimingRequest.LimeLight;
        }
        switch (mAimingRequest) {
            case Odometry:
                double estimatedTimeFrame = 0;
                Pose2d odomToTargetPoint = visionPoseComponent.inverse().transformBy(mFieldToSpeaker);
                double travelDistance = odomToTargetPoint.transformBy(currentOdomToRobot).getTranslation().norm();
                if(currentVelocity.norm() < 1.5){
                estimatedTimeFrame = mShotTimeMap.getInterpolated(new InterpolatingDouble(travelDistance)).value; //TODO
                }
                Pose2d poseAtTimeFrame = RobotState.getInstance().getPredictedPoseFromOdometry(estimatedTimeFrame).rotateBy(currentOdomToRobot.getRotation().inverse());
                Pose2d futureOdomToTargetPoint = poseAtTimeFrame.inverse().transformBy(odomToTargetPoint).inverse();
                Rotation2d targetRotation = futureOdomToTargetPoint.getTranslation().getAngle().inverse();
                targetPose = new Pose2d(futureOdomToTargetPoint.getTranslation(), targetRotation);
                break;
            case LimeLight:

        }
        headingController.setTargetHeading(targetPose.getRotation());
        double rotationOutput = headingController.updateRotationCorrection(currentOdomToRobot.getRotation(), timeStamp);

        targetPose = new Pose2d(
            targetPose.getTranslation(),
            Rotation2d.fromDegrees(rotationOutput*1.75)
            );
        


        return targetPose;
    }


}