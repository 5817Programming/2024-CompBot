// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.wcp.frc.Constants;
import com.wcp.frc.Ports;
import com.wcp.frc.subsystems.Requests.Request;
import com.wcp.lib.TalonDefaultConfig;

import edu.wpi.first.wpilibj.Timer;

public class Pivot extends Subsystem {
  private PeriodicIO mPeriodicIO = new PeriodicIO();
  private TalonFX pivotMotor = new TalonFX(Ports.Pivot);
  private TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
  private State currentState;
  private boolean stateChanged;
  private boolean hasPiece;
  private double timeEnteredState = 0;

  public static Pivot instance = null;

  public static Pivot getInstance() {// if doesnt have an instance of swerve will make a new one
    if (instance == null)
      instance = new Pivot();
    return instance;
  }

  /** Creates a new pivot. */
  public Pivot() {
    configMotors();
    currentState = State.MAXDOWN;
    stateChanged = false;
    hasPiece = false;
  }

  public enum ControlMode {
    MotionMagic,
    Percent,
  }

  public enum State {
    AMP(Constants.PivotConstants.AMP),
    SPEAKER(Constants.PivotConstants.SPEAKER),
    TRANSFER(Constants.PivotConstants.SPEAKER),
    TRAP(Constants.PivotConstants.SPEAKER),
    MAXUP(Constants.PivotConstants.MAX_UP),
    MAXDOWN(Constants.PivotConstants.MAX_DOWN),
    SHOOTING(Constants.PivotConstants.MAX_DOWN);

    double output = 0;

    State(double output){
      this.output = output;
    }
  }

  public void setRamp(double rampTime) {
    pivotMotor.getConfigurator().refresh(pivotConfig);
    pivotConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampTime;
    pivotMotor.getConfigurator().apply(pivotConfig);
  }

  public void configMotors() {
    pivotConfig = TalonDefaultConfig.driveConfigs();
    pivotMotor.getConfigurator().apply(pivotConfig);
  }

  public void setMotionMagic(double position){
    mPeriodicIO.rotationControlMode = ControlMode.MotionMagic;
    mPeriodicIO.rotationDemand = position;
  }

  public void setPivotPercent(double percentage) {
    mPeriodicIO.rotationControlMode = ControlMode.Percent;
    mPeriodicIO.rotationDemand = percentage;
  }

  public void setState(State state) {
    if (state != currentState)
      stateChanged = true;
    currentState = state;
    timeEnteredState = Timer.getFPGATimestamp();
  }

  public void conformToState(State state) {
    setState(state);
    setPivotPercent(state.output);
  }

  public void motionMagic(){
    pivotMotor.setControl(new MotionMagicVoltage(mPeriodicIO.rotationDemand));
  }

  public void setPercent(){
    pivotMotor.setControl(new DutyCycleOut(mPeriodicIO.rotationDemand, true, false, false, false));
     
  }

  public Request stateRequest(State state) {
    return new Request() {

      @Override
      public void act() {
        conformToState(state);
      }
    };
  }

  public Request setpivotPercentRequest(double percentage) {
    return new Request() {

      @Override
      public void act() {
        setPivotPercent(percentage);
      }

    };

  }

  @Override
  public void update() {
    double currentTime = Timer.getFPGATimestamp();
    switch (currentState) {
      case AMP:
        break;
      case SPEAKER:
      
    }

  }

  public Request hasPieceRequest(){
    return new Request() {
      @Override
        public boolean isFinished() {
            return !stateChanged && hasPiece;
        }
    };
  }

  public double getStatorCurrent() {
    return mPeriodicIO.statorCurrent;
  }

  @Override
  public void writePeriodicOutputs() {
    mPeriodicIO.rotationPosition = pivotMotor.getPosition().getValueAsDouble();
    mPeriodicIO.velocity = pivotMotor.getVelocity().getValueAsDouble();
    mPeriodicIO.statorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void readPeriodicInputs() {
    switch (mPeriodicIO.rotationControlMode) {
      case MotionMagic:
        motionMagic();
        break;
      case Percent:
        setPercent();
        break;
    }
  }

  @Override
  public void outputTelemetry() {

  }

  @Override
  public void stop() {
    setpivotPercentRequest(0);
  }

  public static class PeriodicIO {
    ControlMode rotationControlMode = ControlMode.MotionMagic;
    double rotationPosition = 0;
    double velocity = 0;
    double statorCurrent = 0;

    double rotationDemand = 0.0;
  }
}
