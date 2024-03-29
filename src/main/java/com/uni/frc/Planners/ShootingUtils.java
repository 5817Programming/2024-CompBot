// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.Planners;

import org.littletonrobotics.junction.Logger;

import com.uni.frc.Constants;
import com.uni.frc.Constants.FieldConstants;
import com.uni.frc.Constants.ShooterConstants;
import com.uni.frc.subsystems.RobotState;
import com.uni.frc.subsystems.Shooter;
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
        public double compensatedDistance;

        public double compensatedDesiredPivotAngle;

        public double compensatedDesiredShooterSpeed;



        public ShootingParameters(
           double compensatedDistance,
           double compensatedDesiredPivotAngle,
           double compensatedDesiredShooterSpeed
        ){
         this.compensatedDistance = compensatedDistance;
         this.compensatedDesiredPivotAngle = compensatedDesiredPivotAngle;
         this.compensatedDesiredShooterSpeed = compensatedDesiredShooterSpeed;
        }
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
        Pose2d poseAtTimeFrame = RobotState.getInstance().getPredictedPose(lookahead_time);
        Translation2d futureOdomToTargetPoint = poseAtTimeFrame.getTranslation().translateBy(targetPose.getTranslation().inverse());
        double compensatedDistance = futureOdomToTargetPoint.norm();
        double compensatedPivotAngle;
        if(manual)
            compensatedPivotAngle = 55;
        else
            compensatedPivotAngle = getPivotAngle(compensatedDistance, pivotAngleTreeMap);
        double compensatedDesiredShooterSpeed = velocityTreeMap.getInterpolated(new InterpolatingDouble(compensatedDistance)).value;
        return new ShootingParameters(
            compensatedDistance, 
            compensatedPivotAngle,
            compensatedDesiredShooterSpeed
        );
    }

    private static double getPivotAngle(double Distance, InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> dropMap){
        double x = FieldConstants.kSpeakerHeight - ShooterConstants.kShooterHeight;
        double desiredAngle = Math.atan(x/Distance) - ShooterConstants.kShooterZeroAngle;
        double desiredAngleWithDrop = desiredAngle + dropMap.getInterpolated(new InterpolatingDouble(Distance)).value;
        return desiredAngleWithDrop;
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
