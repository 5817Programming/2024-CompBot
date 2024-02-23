 package com.uni.frc.subsystems;

  import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
  import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
 import com.uni.frc.Ports;
 import com.uni.frc.CompConstants.PivotConstants;
 import com.uni.frc.subsystems.Requests.Request;
 import com.uni.lib.TalonConfigs;


  public class Pivot extends Subsystem {
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private TalonFX pivotMotor1 = new TalonFX(Ports.Pivot1,"Minivore");
    private TalonFX pivotMotor2 = new TalonFX(Ports.Pivot2,"Minivore");

    private CANcoder encoder = new CANcoder(Ports.PivotEncoder,"Minivore");
    private TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    private State currentState;
    private boolean stateChanged;


    public static Pivot instance = null;

    public static Pivot getInstance() {
      if (instance == null)
        instance = new Pivot();
      return instance;
    }

    /** Creates a new pivot. */
    public Pivot() {
      configMotors();
      currentState = State.MAXDOWN;
      stateChanged = false;
    }

    public enum ControlMode {
      MotionMagic,
      Percent,
    }

    public enum State {
      AMP(PivotConstants.AMP),
      SPEAKER(PivotConstants.SPEAKER),
      TRANSFER(PivotConstants.SPEAKER),
      TRAP(PivotConstants.SPEAKER),
      MAXUP(PivotConstants.MAX_UP),
      MAXDOWN(PivotConstants.MAX_DOWN),
      SHOOTING(PivotConstants.MAX_DOWN);

      double output = 0;
      State(double output){
        this.output = output;
      }
    }

    public void resetToAbsolute(){
      double currentAngle =getAbsolutePosition()/360;
      pivotMotor1.setPosition(currentAngle);

      mPeriodicIO.rotationDemand = currentAngle;
    }

    public void setRamp(double rampTime) {
      pivotMotor1.getConfigurator().refresh(pivotConfig);
      pivotConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampTime;
      pivotMotor1.getConfigurator().apply(pivotConfig);
    }

    public double getAbsolutePosition(){
      return encoder.getAbsolutePosition().getValueAsDouble()*360;
    }

    public void configMotors() {
      pivotConfig = TalonConfigs.pivotConfigs();
      pivotMotor1.getConfigurator().apply(pivotConfig);
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
    }

    public boolean atTarget(){
      return Math.abs(currentState.output) - Math.abs(mPeriodicIO.rotationPosition) <1;
    }

    public void conformToState(State state) {
      setState(state);
      setMotionMagic(state.output);
    }
    public void conformToState(double Override) {
      setState(State.SHOOTING);
      setMotionMagic(Override);
    }

    public void motionMagic(){
      pivotMotor1.setControl(new MotionMagicVoltage(mPeriodicIO.rotationDemand));
      // pivotMotor2.setControl(new Follower(Ports.Pivot1, false));

    }

    public void setPercent(){
      pivotMotor1.setControl(new DutyCycleOut(mPeriodicIO.rotationDemand, true, false, false, false));
    }

    public Request stateRequest(State state) {
      return new Request() {

        @Override
        public void act() {
          conformToState(state);
        }
      };
    }

    public Request stateRequest(Double position) {
      return new Request() {

        @Override
        public void act() {
          conformToState(position);
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
      mPeriodicIO.rotationPosition = pivotMotor1.getPosition().getValueAsDouble();
      mPeriodicIO.velocity = pivotMotor1.getVelocity().getValueAsDouble();
      mPeriodicIO.statorCurrent = pivotMotor1.getStatorCurrent().getValueAsDouble();
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
      Logger.recordOutput("Pivot Position", pivotMotor1.getPosition().getValue());
      Logger.recordOutput("Pivot Absolute Position",encoder.getAbsolutePosition().getValue()*360);

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

      double rotationDemand = 0;
    }
  }
