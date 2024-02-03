// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc;

import java.util.Arrays;
import java.util.HashMap;

import org.littletonrobotics.junction.LoggedRobot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.wcp.frc.subsystems.RobotState;
import com.wcp.frc.subsystems.RobotStateEstimator;
import com.wcp.frc.subsystems.SuperStructure;
import com.wcp.frc.subsystems.Swerve.SwerveDrive;
import com.wcp.frc.subsystems.Swerve.SwerveDrive.State;
import com.wcp.frc.subsystems.Vision.ObjectLimeLight;
import com.wcp.frc.subsystems.Vision.OdometryLimeLight;
import com.wcp.frc.subsystems.gyros.Gyro;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.motion.PathFollower;
import com.wcp.frc.Autos.AutoBase;
import com.wcp.frc.Autos.M5;
import com.wcp.frc.Autos.M5Safe;
import com.wcp.frc.Autos.M6;
import com.wcp.frc.Autos.M7;
import com.wcp.frc.Autos.NS3;
import com.wcp.frc.Autos.NS5;
import com.wcp.frc.Autos.S3;
import com.wcp.frc.Autos.S5;
import com.wcp.frc.Autos.Shoot;

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
  
  Gyro pigeon;
  public SendableChooser<AutoBase> autoChooser = new SendableChooser<>();

HashMap<String,AutoBase> autos = new HashMap<String,AutoBase>();
  @Override
  public void robotInit() {
    // autos.put("M7", new M7());
    // autos.put("M6", new M6());
    // autos.put("M5", new M5());
    // autos.put("M5 safe",new M5Safe());
    // autos.put("NS5", new NS5());
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
    
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging

    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may

    swerve = SwerveDrive.getInstance();
    controls = Controls.getInstance();
    vision = OdometryLimeLight.getInstance();
    swerve.zeroModules();
    subsystemManager = new SubsystemManager();
    subsystemManager.addSystems(Arrays.asList(
        SwerveDrive.getInstance(),
        SuperStructure.getInstance(), 
        OdometryLimeLight.getInstance(),
        RobotStateEstimator.getInstance(),
        ObjectLimeLight.getInstance()       // Shooter.getInstance()
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

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */

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
    SuperStructure.getInstance().idleState();
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

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    DevControls.getInstance().update();

  }
}
