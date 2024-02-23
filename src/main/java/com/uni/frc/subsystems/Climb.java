 package com.uni.frc.subsystems;

  import javax.sound.sampled.Port;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
  import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
  import com.ctre.phoenix6.hardware.TalonFX;
 import com.uni.frc.Ports;
import com.uni.frc.CompConstants.ElevatorConstants;
 import com.uni.frc.subsystems.Requests.Request;
 import com.uni.lib.TalonConfigs;


  public class Climb extends Subsystem {
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private TalonFX elevatorMotor1 = new TalonFX(Ports.elevatorMotor1);
    private TalonFX elevatorMotor2 = new TalonFX(Ports.elevatorMotor2);
    

    private TalonFXConfiguration elevator1Config = TalonConfigs.elevatorConfigs();

    private State currentState;



    public static Climb instance = null;

    public static Climb getInstance() {
      if (instance == null)
        instance = new Climb();
      return instance;
    }

    /** Creates a new pivot. */
    public Climb() {
      configMotors();

    }

    public enum ControlMode {
      MotionMagic,
      Percent,
    }

    public enum State {
      STOW(ElevatorConstants.STOW),
      OVERCHAIN(ElevatorConstants.OVERCHAIN),
      ONCHAIN(ElevatorConstants.ONCHAIN),
      MAXUP(ElevatorConstants.MAX_UP),
      MAXDOWN(ElevatorConstants.MAX_DOWN);

      double output = 0;

      State(double output){
        this.output = output;
      }
    }

    public void setRamp(double rampTime) {
      elevatorMotor1.getConfigurator().refresh(elevator1Config);
      elevator1Config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampTime;
      elevatorMotor1.getConfigurator().apply(elevator1Config);
    }

    public void configMotors() {
      elevator1Config = TalonConfigs.pivotConfigs();
      elevatorMotor1.getConfigurator().apply(elevator1Config);
    }

    public void setMotionMagic(double position){
      mPeriodicIO.rotationControlMode = ControlMode.MotionMagic;
      mPeriodicIO.rotationDemand = position;
    }

    public void setPivotPercent(double percentage) {
      mPeriodicIO.rotationControlMode = ControlMode.Percent;
      mPeriodicIO.rotationDemand = percentage;
    }


    public boolean atTarget(){
      return Math.abs(currentState.output) - Math.abs(mPeriodicIO.rotationPosition) <1;
    }

    public void conformToState(State state) {
      setMotionMagic(state.output);
    }
    public void conformToState(double Override) {
      setMotionMagic(Override);
    }

    public void motionMagic(){
      elevatorMotor1.setControl(new MotionMagicVoltage(mPeriodicIO.rotationDemand));
      elevatorMotor2.setControl(new Follower(Ports.elevatorMotor1, false));//TODO MAYBE OPPOSE
    }   

    public void setPercent(){
      elevatorMotor1.setControl(new DutyCycleOut(mPeriodicIO.rotationDemand, true, false, false, false));
     
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
              return  atTarget();
          }
      };
    }

    public double getStatorCurrent() {
      return mPeriodicIO.statorCurrent;
    }

    @Override
    public void writePeriodicOutputs() {
      mPeriodicIO.rotationPosition = elevatorMotor1.getPosition().getValueAsDouble();
      mPeriodicIO.velocity = elevatorMotor1.getVelocity().getValueAsDouble();
      mPeriodicIO.statorCurrent = elevatorMotor1.getStatorCurrent().getValueAsDouble();
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