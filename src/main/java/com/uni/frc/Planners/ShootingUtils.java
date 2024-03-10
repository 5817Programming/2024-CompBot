// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.Planners;

import org.littletonrobotics.junction.Logger;

import com.uni.frc.Constants;
import com.uni.frc.subsystems.RobotState;
import com.uni.lib.geometry.Pose2d;
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

        public double uncompensatedPivotAngleError;
        public double uncompensatedDesiredPivotAngle;
        public double compensatedDesiredPivotAngle;
        public double compensatedPivotAngleError;

        public double uncompensatedDesiredShooterSpeed;
        public double compensatedDesiredShooterSpeed;
        public double uncompensatedShooterSpeedError;
        public double compensatedShooterSpeedError;

        public double desiredSpin;

        public ShootingParameters(
           double effectiveDistance,
           double compensatedDistance,
           double compensatedDesiredPivotAngle,
           double uncompensatedDesiredPivotAngle,
           double compensatedDesiredPivotAngleError,
           double uncompensatedDesiredPivotAngleError,
           double compensatedDesiredShooterSpeed,
           double uncompensatedDesiredShooterSpeed,
           double compensatedDesiredShooterSpeedError,
           double uncompensatedDesiredShooterSpeedError,
           double desiredSpin
        ){
         this.effectiveDistance = effectiveDistance;
         this.compensatedDistance = compensatedDistance;
         this.compensatedDesiredPivotAngle = compensatedDesiredPivotAngle;
         this.uncompensatedDesiredPivotAngle = uncompensatedDesiredPivotAngle;
         this.compensatedPivotAngleError = compensatedDesiredPivotAngleError;
         this.uncompensatedPivotAngleError = uncompensatedDesiredPivotAngleError;
         this.compensatedDesiredShooterSpeed = compensatedDesiredShooterSpeed;
         this.uncompensatedDesiredShooterSpeed = uncompensatedDesiredShooterSpeed;
         this.compensatedShooterSpeedError = compensatedDesiredShooterSpeedError;
         this.uncompensatedShooterSpeedError = uncompensatedDesiredShooterSpeedError;
         this.desiredSpin = desiredSpin;

        }
    }

    public static ShootingParameters getShootingParameters(
        Pose2d currentPose, 
        Pose2d targetPose,
        double pivotAngle, 
        double shooterVelovity, 
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

        Logger.recordOutput("Compensated Position", poseAtTimeFrame.toWPI());
        Logger.recordOutput("Time", lookahead_time);

        double compensatedDistance = compensatedShooterToTarget.getTranslation().norm();
        
        
        double desiredPivotAngle = pivotAngleTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value+.01+.006;
        double compensatedPivotAngle = pivotAngleTreeMap.getInterpolated(new InterpolatingDouble(compensatedDistance)).value;
        double uncompensatedDesiredPivotAngleError = Math.abs(desiredPivotAngle - pivotAngle);
        double compensatedDesiredPivotAngleError = Math.abs(compensatedPivotAngle - pivotAngle);

        double uncompensatedDesiredShooterSpeed = velocityTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value;
        double compensatedDesiredShooterSpeed = velocityTreeMap.getInterpolated(new InterpolatingDouble(compensatedDistance)).value;
        double uncompensatedShooterSpeedError = Math.abs(uncompensatedDesiredShooterSpeed - shooterVelovity);
        double compensatedShooterSpeedError = Math.abs(uncompensatedDesiredShooterSpeed - shooterVelovity);

        double desiredSpin = 0; //TODO
        return new ShootingParameters(
            effectiveDistance, 
            compensatedDistance, 
            compensatedPivotAngle,
            desiredPivotAngle,
            compensatedDesiredPivotAngleError,
            uncompensatedDesiredPivotAngleError,
            compensatedDesiredShooterSpeed, 
            uncompensatedDesiredShooterSpeed, 
            compensatedShooterSpeedError,
            uncompensatedShooterSpeedError, 
            desiredSpin);
    }
    public static ShootingParameters getShootingParameters(
        double pivotOffset,
        Pose2d currentPose, 
        Pose2d targetPose,
        double pivotAngle, 
        double shooterVelovity, 
        double kShotTime, 
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> pivotAngleTreeMap,
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> velocityTreeMap,
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shotTimeTreeMap,
        Twist2d currentVelocity,
        boolean manual){
         Pose2d robotToTarget = Pose2d.fromTranslation(targetPose.getTranslation().translateBy(currentPose.getTranslation().inverse()));
        double effectiveDistance = robotToTarget.getTranslation().norm();
        double lookahead_time = shotTimeTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value;
        Logger.recordOutput("effective distance", effectiveDistance);
        Pose2d poseAtTimeFrame = RobotState.getInstance().getPredictedPoseFromOdometry(lookahead_time+.004).rotateBy(currentPose.getRotation());
        Pose2d compensatedShooterToTarget = Pose2d.fromTranslation(targetPose.getTranslation().translateBy(poseAtTimeFrame.getTranslation().inverse()));

        Logger.recordOutput("Compensated Position", poseAtTimeFrame.toWPI());
        Logger.recordOutput("Time", lookahead_time);

        double compensatedDistance = compensatedShooterToTarget.getTranslation().norm();
        
        double desiredPivotAngle;
        if(manual)
            desiredPivotAngle = 0.083-.125;
        else{
            desiredPivotAngle = pivotAngleTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value+.01+pivotOffset;
            Logger.recordOutput("Desired Pivot Angle", desiredPivotAngle);
        }
        double compensatedPivotAngle = pivotAngleTreeMap.getInterpolated(new InterpolatingDouble(compensatedDistance)).value;
        double uncompensatedDesiredPivotAngleError = Math.abs(desiredPivotAngle - pivotAngle);
        double compensatedDesiredPivotAngleError = Math.abs(compensatedPivotAngle - pivotAngle);
        double uncompensatedDesiredShooterSpeed = velocityTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value;
        double compensatedDesiredShooterSpeed = velocityTreeMap.getInterpolated(new InterpolatingDouble(compensatedDistance)).value;
        double uncompensatedShooterSpeedError = Math.abs(uncompensatedDesiredShooterSpeed - shooterVelovity);
        double compensatedShooterSpeedError = Math.abs(uncompensatedDesiredShooterSpeed - shooterVelovity);

        double desiredSpin = 0; //TODO
        return new ShootingParameters(
            effectiveDistance, 
            compensatedDistance, 
            compensatedPivotAngle,
            desiredPivotAngle,
            compensatedDesiredPivotAngleError,
            uncompensatedDesiredPivotAngleError,
            compensatedDesiredShooterSpeed, 
            uncompensatedDesiredShooterSpeed, 
            compensatedShooterSpeedError,
            uncompensatedShooterSpeedError, 
            desiredSpin);
    }

    public static boolean pivotAtSetpoint(ShootingParameters shootingParameters, double deadBand, boolean shootOnMove){
        if(shootOnMove)
            return shootingParameters.compensatedPivotAngleError < deadBand;
        else
            return shootingParameters.uncompensatedPivotAngleError < deadBand;
    }   

    public static boolean shooterAtSetpoint(ShootingParameters shootingParameters, double deadBand, boolean shootOnMove){
        if(shootOnMove)
            return shootingParameters.compensatedShooterSpeedError < deadBand;
        else
            return shootingParameters.uncompensatedShooterSpeedError < deadBand;    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getPivotMap(boolean isAutonomous){
        return Constants.PivotConstants.kNewPivotShootingMap;
    }
 public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getVelocityMap(boolean isAutonomous){
        return Constants.ShooterConstants.VELOCITY_TREE_MAP;
    }
}
