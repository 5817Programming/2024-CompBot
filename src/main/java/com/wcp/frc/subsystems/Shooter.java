// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import java.lang.constant.Constable;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.wcp.frc.Constants;
import com.wcp.frc.Ports;
import com.wcp.lib.geometry.Translation2d;

public class Shooter extends Subsystem {
  public static Shooter instance = null;
  Swerve swerve;
  PeriodicIO mPeriodicIO = new PeriodicIO();
  TalonFX shooter = new TalonFX(Ports.shooter);


  public static Shooter getInstance() {
      if (instance == null)
          instance = new Shooter();
      return instance;
  }
  /** Creates a new Shooter. */
  public Shooter() {
  }
  public enum ControlMode{
    Percent,
    MotionMagic,
  }

  public enum State{
    OFF(0),
    ON(1),
    IDLE(Constants.ShooterConstants.IDLE),
    HANDOFF(Constants.ShooterConstants.HANDOFF);

    double output = 0;

    State(double output){
      this.output = output;
    }
  }
  public Translation2d getMovingTarget(){
    // return swerve.getVelocity().scale(ShooterConstants.TIME_TREEMAP.getInterpolated(new InterpolatingDouble(swerve.getPose().minus(FieldConstants.Speaker))).value);TODO
    return new Translation2d();
  }

  public void conformToState(State state){
    setPercentOuput(state.output);
  }

  public void setPercentOuput(double output){
    mPeriodicIO.driveControlMode = ControlMode.Percent;
    mPeriodicIO.driveDemand = output;
  }

  @Override
  public void outputTelemetry() {
    
  }

  @Override
  public void stop() {
    
  }

  @Override
  public void writePeriodicOutputs() {
    mPeriodicIO.drivePosition = shooter.getPosition().getValueAsDouble();
    mPeriodicIO.velocity = shooter.getVelocity().getValueAsDouble();
  }

  @Override
  public void readPeriodicInputs() {
    switch (mPeriodicIO.driveControlMode) {
      case Percent:
          runPercentOutput(mPeriodicIO.driveDemand);
        break;
      case MotionMagic:
          runMotionMagic(mPeriodicIO.driveDemand);
    }
  }

  public void runPercentOutput(double percent){
    shooter.setControl(new DutyCycleOut(percent, true, true, true, true));
  }
    public void runMotionMagic(double position){
    shooter.setControl(new MotionMagicVoltage(position));
  }


  public static class PeriodicIO  {
    double drivePosition = 0;
    double velocity = 0;

    ControlMode driveControlMode = ControlMode.MotionMagic;
    double rotationDemand = 0.0;
    double driveDemand = 0.0;
  }
}
