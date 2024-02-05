// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc;

import java.util.Arrays;
import java.util.HashMap;

import org.littletonrobotics.junction.LoggedRobot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.uni.frc.Autos.AutoBase;
import com.uni.frc.Autos.M5;
import com.uni.frc.Autos.M5Safe;
import com.uni.frc.Autos.M6;
import com.uni.frc.Autos.M7;
import com.uni.frc.Autos.NS3;
import com.uni.frc.Autos.NS5;
import com.uni.frc.Autos.S3;
import com.uni.frc.Autos.S5;
import com.uni.frc.Autos.Shoot;
import com.uni.frc.subsystems.Music;
import com.uni.frc.subsystems.RobotState;
import com.uni.frc.subsystems.RobotStateEstimator;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import com.uni.frc.subsystems.Swerve.SwerveDrive.State;
import com.uni.frc.subsystems.Vision.ObjectLimeLight;
import com.uni.frc.subsystems.Vision.OdometryLimeLight;
import com.uni.frc.subsystems.gyros.Gyro;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.motion.PathFollower;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

//https://github.com/Mechanical-Advantage/AdvantageKit/releases/latest/download/AdvantageKit.json
//https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json

public class Robot extends LoggedRobot {

  Controls controls;
  SubsystemManager subsystemManager;
  SwerveDrive swerve;
  double yaw;
  OdometryLimeLight vision;
  Music music;
  Gyro pigeon;
  public SendableChooser<AutoBase> autoChooser = new SendableChooser<>();

HashMap<String,AutoBase> autos = new HashMap<String,AutoBase>();
  @Override
  public void robotInit() {
    autos.put("M7", new M7());
    autos.put("M6", new M6());
    autos.put("M5", new M5());
    autos.put("M5 safe",new M5Safe());
    autos.put("NS5", new NS5());
    // autos.put("NS3", new NS3());
    // autos.put("S5", new S5());
    // autos.put("S3", new S3());
    RobotState.getInstance().resetKalmanFilters();
    for(HashMap.Entry<String, AutoBase> entry : autos.entrySet()) {
      String N = entry.getKey();
      AutoBase A = entry.getValue();
      autoChooser.addOption(N, A);
    }

    SmartDashboard.putData("Autonomous routine", autoChooser);
    new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    swerve = SwerveDrive.getInstance();
    music = Music.getInstance();
    controls = Controls.getInstance();
    vision = OdometryLimeLight.getInstance();
    swerve.zeroModules();
    subsystemManager = new SubsystemManager();
    subsystemManager.addSystems(Arrays.asList(
        SwerveDrive.getInstance(),
        SuperStructure.getInstance(), 
        OdometryLimeLight.getInstance(),
        RobotStateEstimator.getInstance(),
        ObjectLimeLight.getInstance(),
        Music.getInstance()
               // Shooter.getInstance()
        // Intake.getInstance()
        ));

    }

  @Override
  public void robotPeriodic() {
    subsystemManager.updateSubsystems();
    subsystemManager.readSystemsPeriodicInputs();
    subsystemManager.writeSubsystemsPeriodicOutputs();
    subsystemManager.outputSystemsTelemetry();
    CommandScheduler.getInstance().run();
    Logger.recordOutput("timestamp", Timer.getFPGATimestamp());
  }

  double startime;


  @Override
  public void autonomousInit() {

    swerve = SwerveDrive.getInstance();
    swerve.fieldzeroSwerve();
    swerve.zeroModules();

    
    // if(autoChooser.getSelected() != null){
    //   autoChooser.getSelected().runAuto();
    // }else{
    // }
    new M6().runAuto();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    swerve = SwerveDrive.getInstance();
    swerve.fieldzeroSwerve();
    swerve.zeroModules();

    RobotStateEstimator.getInstance().resetOdometry(new Pose2d(15.24,5.54, new Rotation2d()));
    RobotState.getInstance().resetKalmanFilters();
    



  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    controls.update();
  }

  /** This function is called once when the robot is disabled. */

  @Override
  public void disabledInit() {
    subsystemManager.stopSubsystems();
    SuperStructure.getInstance().clearQueues();
    PathFollower.getInstance().resetTimer();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    

  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    music.play("output.chrp");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}