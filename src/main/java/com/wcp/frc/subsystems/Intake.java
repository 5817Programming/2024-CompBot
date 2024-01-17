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


public class Intake extends Subsystem {
  PeriodicIO mPeriodicIO = new PeriodicIO();
  TalonFX intakeMotor = new TalonFX(Ports.intake);
  TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

  public static Intake instance = null;

  public static Intake getInstance() {// if doesnt have an instance of swerve will make a new one
    if (instance == null)
      instance = new Intake();
    return instance;
  }

  /** Creates a new intake. */
  public Intake() {
    configMotors();
  }

  public void configMotors() {
    intakeConfig = TalonDefaultConfig.driveConfigs();
    intakeMotor.getConfigurator().apply(intakeConfig);
  }

  public void intakePercent(double Percentage) {
    mPeriodicIO.driveDemand = Percentage;
  }

  public Request setIntakePercentRequest(double percentage) {
    return new Request() {

      @Override
      public void act() {

        intakePercent(percentage);
      }

    };

  }

  @Override
  public void readPeriodicInputs() {
    intakeMotor.setControl(new DutyCycleOut(mPeriodicIO.driveDemand, true, false, false, false));
  }

  @Override
  public void outputTelemetry() {

  }

  @Override
  public void stop() {
    setIntakePercentRequest(0);
  }

  public static class PeriodicIO {// data
    double rotationPosition = 0;
    double drivePosition = 0;
    double velocity = 0;

    double rotationDemand = 0.0;
    double driveDemand = 0.0;
  }
}
