package com.uni.frc.subsystems;


 import com.ctre.phoenix6.configs.TalonFXConfiguration;
 import com.ctre.phoenix6.controls.VelocityDutyCycle;
 import com.ctre.phoenix6.hardware.TalonFX;
import com.uni.frc.Constants;
import com.uni.frc.Ports;
import com.uni.frc.subsystems.Requests.Request;
import com.uni.lib.TalonConfigs;


 public class Shooter extends Subsystem {
   private PeriodicIO mPeriodicIO = new PeriodicIO();
   private TalonFX shooterMotor1 = new TalonFX(Ports.shooter1);
   private TalonFX shooterMotor2 = new TalonFX(Ports.shooter2);

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

   @Override
   public void writePeriodicOutputs() {
     mPeriodicIO.drivePosition = shooterMotor1.getPosition().getValueAsDouble();
     mPeriodicIO.velocity = shooterMotor1.getVelocity().getValueAsDouble();
     mPeriodicIO.statorCurrent = (shooterMotor1.getStatorCurrent().getValueAsDouble()+shooterMotor2.getStatorCurrent().getValueAsDouble())/2;
   }

   @Override
   public void readPeriodicInputs() {
     shooterMotor1.setControl(new VelocityDutyCycle(mPeriodicIO.driveDemand,
                                                   mPeriodicIO.accelerationDemand,
                                          true,
                                                   Constants.ShooterConstants.FEEDFORWARD,
                                                   0,false,
                                                   false,
                                                   false));
     shooterMotor1.setControl(new VelocityDutyCycle(mPeriodicIO.driveDemand-spinOffset,
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
