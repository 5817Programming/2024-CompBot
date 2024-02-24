package com.uni.frc.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.uni.frc.Constants;
import com.uni.frc.Ports;
import com.uni.frc.subsystems.Requests.Request;
import com.uni.lib.TalonConfigs;

import edu.wpi.first.wpilibj.Timer;

public class Hand extends Subsystem {
  private PeriodicIO mPeriodicIO = new PeriodicIO();
  private TalonFX handMotor = new TalonFX(Ports.Hand);
  private BeamBreak handBeamBreak = new BeamBreak(Ports.handBeamBreak);

  private TalonFXConfiguration handConfig = TalonConfigs.handConfigs();
  public static Hand instance = null;

  public static Hand getInstance() {
    if (instance == null)
      instance = new Hand();
    return instance;
  }

  /** Creates a new shooter. */
  public Hand() {
    configMotors();
  }

  public enum State {
    SHOOTING(0.5),
    TRANSFERING(-0.5),
    REVERSETRANSFER(.5),
    OFF(0);

    double output = 0;

    State(double output) {
      this.output = output;
    }
  }

  public void setRamp(double rampTime) {
    handMotor.getConfigurator().refresh(handConfig);

    handConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampTime;

    handMotor.getConfigurator().apply(handConfig);
  }

  public void configMotors() {
    handConfig = TalonConfigs.shooterConfigs();
    handMotor.getConfigurator().apply(handConfig);

  }

  public void setPercent(double Percentage) {
    mPeriodicIO.driveDemand = Percentage;
  }

  public void setAcceleration(double accelerationDemand) {
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

  public void conformToState(State state) {
    mPeriodicIO.driveDemand = state.output;
  }

  public Request setPercentRequest(double percentage) {
    return new Request() {

      @Override
      public void act() {

        setPercent(percentage);
      }

    };

  }

  public boolean hasPiece() {
    return mPeriodicIO.hasPiece;
  }

  public Request hasPieceRequest(boolean timeout) {
    if (timeout) {
      return new Request() {
        @Override
        public boolean isFinished() {
          return mPeriodicIO.hasPiece;
        }
      };
    }
    return new Request() {
      double startTime;

      @Override
      public void initialize() {
        startTime = Timer.getFPGATimestamp();
      }

      @Override
      public boolean isFinished() {
        return mPeriodicIO.hasPiece || Timer.getFPGATimestamp() - startTime > 3;
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

  }

  public double getStatorCurrent() {
    return mPeriodicIO.statorCurrent;
  }

  @Override
  public void writePeriodicOutputs() {
    mPeriodicIO.drivePosition = handMotor.getPosition().getValueAsDouble();
    mPeriodicIO.velocity = handMotor.getVelocity().getValueAsDouble();
    mPeriodicIO.statorCurrent = (handMotor.getStatorCurrent().getValueAsDouble());
    mPeriodicIO.hasPiece = handBeamBreak.get();
  }

  @Override
  public void readPeriodicInputs() {
    handMotor.setControl(new VelocityDutyCycle(
        mPeriodicIO.driveDemand,
        mPeriodicIO.accelerationDemand,
        true,
        Constants.ShooterConstants.FEEDFORWARD,
        0, false,
        false,
        false));
  }

  @Override
  public void outputTelemetry() {

  }

  @Override
  public void stop() {
    setPercentRequest(0);
  }

  public static class PeriodicIO {
    double drivePosition = 0;
    double velocity = 0;
    boolean hasPiece = false;

    double statorCurrent = 0;
    double accelerationDemand = 0;
    double driveDemand = 0.0;
  }
}
