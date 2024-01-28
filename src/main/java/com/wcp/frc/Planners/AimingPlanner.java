// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.Planners;

import java.util.Optional;
import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

import com.wcp.frc.Constants;
import com.wcp.frc.subsystems.RobotState;
import com.wcp.frc.subsystems.Requests.Request;
import com.wcp.frc.subsystems.Vision.LimeLight.VisionUpdate;
import com.wcp.frc.subsystems.gyros.Pigeon;
import com.wcp.lib.HeadingController;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;

import com.wcp.lib.util.InterpolatingDouble;
import com.wcp.lib.util.InterpolatingTreeMap;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class AimingPlanner {

    private boolean mAimingComplete = false;
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mShotTimeMap = Constants.ShooterConstants.SHOT_TRAVEL_TIME_TREE_MAP;

    private Pose2d mFieldToSpeaker= new Pose2d(16.28,5.54,new Rotation2d());
    private OptionalDouble mStartingTime;
    private Optional<VisionUpdate> mVisionUpdate;
    private AimingRequest mAimingRequest;

    public enum AimingRequest{
        Odometry,
        LimeLight,
    }
    public AimingPlanner(){
    }



    public AimingRequest getAimingRequest(){
        return mAimingRequest;
    }

    public Pose2d updateAiming(double timeStamp, Pose2d currentOdomToRobot, Pose2d visionPoseComponent, AimingRequest request,Optional<VisionUpdate> visionUpdate, HeadingController headingController){
        Pose2d targetPose = new Pose2d();
        mAimingRequest = request;
        if(mAimingRequest == AimingRequest.LimeLight && mVisionUpdate.isEmpty()){
            System.out.println("No Vision Update For Aiming, Switching To Odometry: AimingUtils");
            mAimingRequest = AimingRequest.Odometry;
        }
        switch (mAimingRequest) {
            case Odometry:
                Pose2d odomToTargetPoint = visionPoseComponent.inverse().transformBy(mFieldToSpeaker);
                Logger.recordOutput("odomToTargt", odomToTargetPoint.toWPI());
                double travelDistance = odomToTargetPoint.transformBy(currentOdomToRobot).getTranslation().norm();
                InterpolatingDouble estimatedTimeFrame = mShotTimeMap.getInterpolated(new InterpolatingDouble(travelDistance));
                Pose2d poseAtTimeFrame = RobotState.getInstance().getPredictedPoseFromOdometry(.5).rotateBy(currentOdomToRobot.getRotation().inverse());
                Pose2d futureOdomToTargetPoint = poseAtTimeFrame.inverse().transformBy(odomToTargetPoint).inverse();
                Logger.recordOutput("pose", poseAtTimeFrame.toWPI());

                Rotation2d targetRotation = futureOdomToTargetPoint.getTranslation().getAngle().inverse();
                targetPose = new Pose2d(futureOdomToTargetPoint.getTranslation(), targetRotation);
                break;
            case LimeLight:
                if(mVisionUpdate.isPresent()){
                Pose2d cameraTotag = Pose2d.fromTranslation(mVisionUpdate.get().getCameraToTarget().rotateBy(Constants.VisionConstants.cameraYawOffset));
                Pose2d tagToRobot = Pose2d.fromTranslation(Constants.VisionConstants.ROBOT_TO_CAMERA.transformBy(cameraTotag).getTranslation().rotateBy(currentOdomToRobot.getRotation()));
                targetPose = new Pose2d(tagToRobot.getTranslation(),tagToRobot.getTranslation().getAngle());
                } else{
                    System.out.println("How did you get here: AimingUtils 71");
                }
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
