// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.wcp.frc.Constants;
import com.wcp.frc.Ports;
import com.wcp.frc.subsystems.Requests.Request;
import com.wcp.lib.TalonDefaultConfig;

import edu.wpi.first.wpilibj.Timer;

public class Shooter extends Subsystem {
  private PeriodicIO mPeriodicIO = new PeriodicIO();
  private TalonFX shooterMotor = new TalonFX(Ports.shooter);
  private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
  private State currentState;
  private boolean stateChanged;
  private boolean hasPiece;
  private double timeEnteredState = 0;

  public static Shooter instance = null;

  public static Shooter getInstance() {// if doesnt have an instance of swerve will make a new one
    if (instance == null)
      instance = new Shooter();
    return instance;
  }

  /** Creates a new shooter. */
  public Shooter() {
    configMotors();

    currentState = State.Off;
    stateChanged = false;
    hasPiece = false;
  }

  public enum State {
    Ramping(.5),
    Shooting(1),
    Recieving(1),
    Waiting(1),
    Off(0);

    double output = 0;

    State(double output) {
      this.output = output;
    }
  }

  public void setRamp(double rampTime) {
    shooterMotor.getConfigurator().refresh(shooterConfig);
    shooterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampTime;
    shooterMotor.getConfigurator().apply(shooterConfig);
  }

  public void configMotors() {
    shooterConfig = TalonDefaultConfig.driveConfigs();
    shooterMotor.getConfigurator().apply(shooterConfig);
  }

  public void shooterPercent(double Percentage) {
    mPeriodicIO.driveDemand = Percentage;
  }

  public void setState(State state) {
    if (state != currentState)
      stateChanged = true;
    currentState = state;
    timeEnteredState = Timer.getFPGATimestamp();
  }

  public void conformToState(State state) {
    setState(state);
    setShooterPercentRequest(state.output);
  }
  
  public void setAcceleration(double accelerationDemand){
    mPeriodicIO.accelerationDemand = accelerationDemand;
  }

  public Request stateRequest(State state) {
    return new Request() {

      @Override
      public void act() {
        conformToState(state);
      }
    };
  }
  

  public Request setShooterPercentRequest(double percentage) {
    return new Request() {

      @Override
      public void act() {

        shooterPercent(percentage);
      }

    };

  }

public Request setAccelerationRequest(double percentage) {
    return new Request() {

      @Override
      public void act() {
        setAcceleration(percentage);
      }

    };

  }


  @Override
  public void update() {
    double currentTime = Timer.getFPGATimestamp();
    switch (currentState) {
      case Ramping:
        break;
      case Recieving:
        break;
      case Off:
        break;
      case Shooting:
        break;
      case Waiting:
        break;

    }

  }

  public boolean atTarget(){
    return Math.abs(mPeriodicIO.velocity) - Math.abs(mPeriodicIO.driveDemand) < 300 ;
  }

  public Request atTargetRequest(){
    return new Request() {
      @Override
        public boolean isFinished() {
            return !stateChanged && atTarget();
        }
    };
  }

  public double getStatorCurrent() {
    return mPeriodicIO.statorCurrent;
  }

  @Override
  public void writePeriodicOutputs() {
    mPeriodicIO.drivePosition = shooterMotor.getPosition().getValueAsDouble();
    mPeriodicIO.velocity = shooterMotor.getVelocity().getValueAsDouble();
    mPeriodicIO.statorCurrent = shooterMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void readPeriodicInputs() {
    shooterMotor.setControl(new VelocityDutyCycle(mPeriodicIO.driveDemand,
                                                  mPeriodicIO.accelerationDemand,
                                         true,
                                                  Constants.ShooterConstants.FEEDFORWARD,
                                                  0,false,
                                                  false,
                                                  false));
  }

  @Override
  public void outputTelemetry() {

  }

  @Override
  public void stop() {
    setShooterPercentRequest(0);
  }

  public static class PeriodicIO {// data
    double drivePosition = 0;
    double velocity = 0;
    double statorCurrent = 0;
    double accelerationDemand = 0;
    double driveDemand = 0.0;
  }
}
