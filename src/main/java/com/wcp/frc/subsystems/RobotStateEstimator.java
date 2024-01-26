// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import com.wcp.frc.subsystems.Swerve.SwerveDrive;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Twist2d;
import com.wcp.lib.swerve.SwerveOdometry;

import edu.wpi.first.wpilibj.Timer;



public class RobotStateEstimator extends Subsystem {



  static RobotStateEstimator instance = null;

  private SwerveOdometry mOdometry;
  private SwerveDrive mDrive;
  
  public static RobotStateEstimator getInstance(){
    if(instance == null){
      instance = new RobotStateEstimator();
      return instance;
    }

    return instance;
  }
  /** Creates a new RobotStateEstimator. */
  public RobotStateEstimator() { 
    mDrive = SwerveDrive.getInstance();
    mOdometry = new SwerveOdometry(mDrive.getKinematics(), Pose2d.identity());
  }

  @Override
  public void update(){
    double timeStamp = Timer.getFPGATimestamp();
    mOdometry.updateWithSwerveModuleStates(mDrive.getRobotHeading(), mDrive.getModules(), timeStamp);
    Twist2d measuredVelocity = mOdometry.getVelocity().toTwist2d();
    Twist2d predictedVelocity = mDrive.getSetPoint().toTwist2d();
    RobotState.getInstance().addOdomObservations(timeStamp, mOdometry.getPoseMeters(), measuredVelocity, predictedVelocity);
    mDrive.resetModulePose(mOdometry.getPoseMeters());
  }

  public void resetOdometry(Pose2d initialPose) {
    synchronized(RobotStateEstimator.this){
      mOdometry.resetPosition(initialPose, mDrive.getModules());
      RobotState.getInstance().reset(Timer.getFPGATimestamp(),initialPose);
    }
  } 

  public void resetGyro(Rotation2d rotation){
    synchronized(RobotStateEstimator.this){
      mOdometry.resetPosition(new Pose2d(mOdometry.getPoseMeters().getTranslation(), rotation), mDrive.getModules());
    }
  }

  @Override
  public void outputTelemetry() {
    RobotState.getInstance().outputTelemetry();
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
  }

}
