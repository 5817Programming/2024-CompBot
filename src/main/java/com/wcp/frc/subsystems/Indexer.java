// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.wcp.frc.Ports;
import com.wcp.frc.subsystems.Requests.Request;
import com.wcp.lib.TalonDefaultConfig;


public class Indexer extends Subsystem {
  PeriodicIO mPeriodicIO = new PeriodicIO();
  TalonFX indexerMotor = new TalonFX(Ports.intake);
  TalonFXConfiguration indexConfig = new TalonFXConfiguration();

  public static Indexer instance = null;

  public static Indexer getInstance() {// if doesnt have an instance of swerve will make a new one
    if (instance == null)
      instance = new Indexer();
    return instance;
  }

  /** Creates a new intake. */
  public Indexer() {
    configMotors();
  }

  public void configMotors() {
    indexConfig = TalonDefaultConfig.driveConfigs();
    indexerMotor.getConfigurator().apply(indexConfig);
  }

  public void intakePercent(double Percentage) {
    mPeriodicIO.driveDemand = Percentage;
  }

  public Request setIndexPercentRequest(double percentage) {
    return new Request() {

      @Override
      public void act() {

        intakePercent(percentage);
      }

    };

  }

  @Override
  public void readPeriodicInputs() {
    indexerMotor.setControl(new DutyCycleOut(mPeriodicIO.driveDemand, true, false, false, false));
  }

  @Override
  public void outputTelemetry() {

  }

  @Override
  public void stop() {
    setIndexPercentRequest(0);
  }

  public static class PeriodicIO {// data
    double rotationPosition = 0;
    double drivePosition = 0;
    double velocity = 0;

    double rotationDemand = 0.0;
    double driveDemand = 0.0;
  }
}
