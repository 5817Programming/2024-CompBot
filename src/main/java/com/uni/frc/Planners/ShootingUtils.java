// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.Planners;

import org.littletonrobotics.junction.Logger;

import com.uni.frc.Constants;
import com.uni.frc.subsystems.RobotState;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Translation2d;
import com.uni.lib.geometry.Twist2d;
import com.uni.lib.util.InterpolatingDouble;
import com.uni.lib.util.InterpolatingTreeMap;


/** Add your docs here. */
public class ShootingUtils {
    

    public enum NoteState {
        NEW,
        MEDIUM,
        OLD
    }

    public static class ShootingParameters{
        public double effectiveDistance;
        public double compensatedDistance;

        public double uncompensatedDesiredPivotAngle;
        public double compensatedDesiredPivotAngle;

        public double uncompensatedDesiredShooterSpeed;
        public double compensatedDesiredShooterSpeed;



        public ShootingParameters(
           double effectiveDistance,
           double compensatedDistance,
           double compensatedDesiredPivotAngle,
           double uncompensatedDesiredPivotAngle,

           double compensatedDesiredShooterSpeed,
           double uncompensatedDesiredShooterSpeed

        ){
         this.effectiveDistance = effectiveDistance;
         this.compensatedDistance = compensatedDistance;
         this.compensatedDesiredPivotAngle = compensatedDesiredPivotAngle;
         this.uncompensatedDesiredPivotAngle = uncompensatedDesiredPivotAngle;
         this.compensatedDesiredShooterSpeed = compensatedDesiredShooterSpeed;
         this.uncompensatedDesiredShooterSpeed = uncompensatedDesiredShooterSpeed;


        }
    }

    public static ShootingParameters getShootingParameters(
        Pose2d currentPose, 
        Pose2d targetPose,
        double kShotTime, 
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> pivotAngleTreeMap,
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> velocityTreeMap,
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shotTimeTreeMap,
        Twist2d currentVelocity){
         Pose2d robotToTarget = Pose2d.fromTranslation(targetPose.getTranslation().translateBy(currentPose.getTranslation().inverse()));
        double effectiveDistance = robotToTarget.getTranslation().norm();
        double lookahead_time = shotTimeTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value;
        Logger.recordOutput("effective distance", effectiveDistance);
        Pose2d poseAtTimeFrame = RobotState.getInstance().getPredictedPoseFromOdometry(lookahead_time+.004).rotateBy(currentPose.getRotation());
        Pose2d compensatedShooterToTarget = Pose2d.fromTranslation(targetPose.getTranslation().translateBy(poseAtTimeFrame.getTranslation().inverse()));

        Logger.recordOutput("Time", lookahead_time);

        double compensatedDistance = compensatedShooterToTarget.getTranslation().norm();
                Logger.recordOutput("Compensated Position", poseAtTimeFrame.toWPI());

        
        double desiredPivotAngle = pivotAngleTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value;
        double compensatedPivotAngle = pivotAngleTreeMap.getInterpolated(new InterpolatingDouble(compensatedDistance)).value;


        double uncompensatedDesiredShooterSpeed = velocityTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value;
        double compensatedDesiredShooterSpeed = velocityTreeMap.getInterpolated(new InterpolatingDouble(compensatedDistance)).value;


        return new ShootingParameters(
            effectiveDistance, 
            compensatedDistance, 
            compensatedPivotAngle,
            desiredPivotAngle,
            compensatedDesiredShooterSpeed, 
            uncompensatedDesiredShooterSpeed
);
    }
    public static ShootingParameters getShootingParameters(
        double pivotOffset,
        Pose2d currentPose, 
        Pose2d targetPose,
        double kShotTime, 
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> pivotAngleTreeMap,
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> velocityTreeMap,
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shotTimeTreeMap,
        Twist2d currentVelocity,
        boolean manual){
         Pose2d robotToTarget = Pose2d.fromTranslation(targetPose.getTranslation().translateBy(currentPose.getTranslation().inverse()));
        double effectiveDistance = robotToTarget.getTranslation().norm();
        double lookahead_time = shotTimeTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value;
        Pose2d poseAtTimeFrame = RobotState.getInstance().getPredictedPose(lookahead_time*1.3);
        Translation2d futureOdomToTargetPoint = poseAtTimeFrame.getTranslation().translateBy(targetPose.getTranslation().inverse());

        Logger.recordOutput("speakerpose", poseAtTimeFrame.toWPI());
        Logger.recordOutput("Time", lookahead_time);

        double compensatedDistance = futureOdomToTargetPoint.norm();
        Logger.recordOutput("Compensated Distance", compensatedDistance);

        
        double desiredPivotAngle;
        if(manual)
            desiredPivotAngle = 55;
        else{
            desiredPivotAngle = pivotAngleTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value;
            Logger.recordOutput("Desired Pivot Angle", desiredPivotAngle);
        }
        double compensatedPivotAngle = pivotAngleTreeMap.getInterpolated(new InterpolatingDouble(compensatedDistance)).value;
        double uncompensatedDesiredShooterSpeed = velocityTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value;
        double compensatedDesiredShooterSpeed = velocityTreeMap.getInterpolated(new InterpolatingDouble(compensatedDistance)).value;
       
        return new ShootingParameters(
            effectiveDistance, 
            compensatedDistance, 
            compensatedPivotAngle,
            desiredPivotAngle,
            compensatedDesiredShooterSpeed, 
            uncompensatedDesiredShooterSpeed

            );
    }



    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getPivotMap(boolean lob){
        System.out.println(lob);
        if
        (lob)
            return Constants.PivotConstants.LobAngleMap;
        return Constants.PivotConstants.SpeakerAngleMap;
    }
 public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getVelocityMap(boolean lob){
        if(lob)
            return Constants.ShooterConstants.LOB_VELOCITY_TREE_MAP;
        return Constants.ShooterConstants.SPEAKER_VELOCITY_TREE_MAP;
    }
}
