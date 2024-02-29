package com.uni.frc.subsystems;


 import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
 import com.ctre.phoenix6.hardware.TalonFX;
import com.uni.frc.Constants;
import com.uni.frc.Ports;
import com.uni.frc.subsystems.Requests.Request;
import com.uni.lib.TalonConfigs;


 public class Shooter extends Subsystem {
   private PeriodicIO mPeriodicIO = new PeriodicIO();
   private TalonFX shooterMotor1 = new TalonFX(Ports.shooter1, "Minivore");
   private TalonFX shooterMotor2 = new TalonFX(Ports.shooter2, "Minivore");
   public State currentState = State.IDLE;
   private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
   private double spinOffset = 0;
   public static Shooter instance = null;

   public static Shooter getInstance() {
     if (instance == null)
       instance = new Shooter();
     return instance;
   }

   /** Creates a new shooter. */
   public Shooter() {
     configMotors();
   }

   public enum State{
    PARTIALRAMP(.7),
    SHOOTING(1),
    TRANSFER(0.5),
    REVERSETRANSFER(-.5),
    IDLE(0);

    double output = 0;
    State(double output){
        this.output = output;
    }
   }

   public void setRamp(double rampTime) {
     shooterMotor1.getConfigurator().refresh(shooterConfig);
     shooterMotor2.getConfigurator().refresh(shooterConfig);

     shooterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampTime;

     shooterMotor1.getConfigurator().apply(shooterConfig);
     shooterMotor2.getConfigurator().apply(shooterConfig);
   }

   public void configMotors() {
     shooterConfig = TalonConfigs.shooterConfigs();
     shooterMotor1.getConfigurator().apply(shooterConfig);
     shooterMotor2.getConfigurator().apply(shooterConfig);

   }

   public void setPercent(double Percentage) {
     currentState = State.SHOOTING;
     mPeriodicIO.driveDemand = Percentage;
   }


  
   public void setAcceleration(double accelerationDemand){
     mPeriodicIO.accelerationDemand = accelerationDemand;
   }


  

   public Request setPercentRequest(double percentage) {
     return new Request() {

       @Override
       public void act() {

         setPercent(percentage);
       }

     };

   }
   public void conformToState(State state){
    currentState = state;
    setPercent(state.output);
   }
 public Request stateRequest(State state){
  return new Request() {
    @Override
    public void act() {
        setPercent(state.output);
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
   public void setSpin(double spinOffset){
    this.spinOffset = spinOffset;
   }




   public double getStatorCurrent() {
     return mPeriodicIO.statorCurrent;
   }

   public Request atTargetRequest(){
    return new Request(){
      @Override
      public boolean isFinished() {
          // TODO Auto-generated method stub
          return Math.abs(shooterMotor1.getVelocity().getValueAsDouble()-(80*.8)) < 3;
      }
    };
   }

   @Override
   public void writePeriodicOutputs() {
     mPeriodicIO.drivePosition = shooterMotor1.getPosition().getValueAsDouble();
     mPeriodicIO.velocity = shooterMotor1.getVelocity().getValueAsDouble();
     mPeriodicIO.statorCurrent = (shooterMotor1.getStatorCurrent().getValueAsDouble()+shooterMotor2.getStatorCurrent().getValueAsDouble())/2;
   }

   @Override
   public void readPeriodicInputs() {
      shooterMotor1.setControl(new DutyCycleOut(mPeriodicIO.driveDemand).withEnableFOC(true));//TODO
      shooterMotor2.setControl(new DutyCycleOut(-mPeriodicIO.driveDemand*0.7).withEnableFOC(true));
      
      
      
      // hEnableFOC(true));

      // shooterMotor1.setControl(new MotionMagicVelocityDutyCycle(mPeriodicIO.driveDemand).withEnableFOC(true));
      // shooterMotor2.setControl(new MotionMagicVelocityDutyCycle(-mPeriodicIO.driveDemand).withEnableFOC(true));
   }

   @Override
   public void outputTelemetry() {
      Logger.recordOutput("zVelocity",shooterMotor1.getVelocity().getValueAsDouble());
   }

   @Override
   public void stop() {
     setPercentRequest(0);
   }

   public static class PeriodicIO {
     double drivePosition = 0;
     double velocity = 0;
     double statorCurrent = 0;
     double accelerationDemand = 0;
     double driveDemand = 0.0;
   }
 }
