package com.uni.frc.subsystems;



 import com.ctre.phoenix6.configs.TalonFXConfiguration;
 import com.ctre.phoenix6.controls.DutyCycleOut;
 import com.ctre.phoenix6.hardware.TalonFX;
import com.uni.frc.Ports;
import com.uni.frc.subsystems.Requests.Request;
import com.uni.lib.TalonConfigs;

 public class Indexer extends Subsystem {

   private PeriodicIO mPeriodicIO = new PeriodicIO();
   private TalonFX indexerMotor = new TalonFX(Ports.Intake);
   private BeamBreak indexerBeamBreak = new BeamBreak(Ports.IndexerBeamBreakPort);
   private TalonFXConfiguration indexerConfig = new TalonFXConfiguration();


   public static Indexer instance = null;

   public static Indexer getInstance() { 
     if (instance == null)
       instance = new Indexer();
     return instance;
   }

   /** Creates a new intake. */
   public Indexer() {
     configMotors();
   }
   public enum State{
    OFF(0),
    RECIEVING(0.5),
    TRANSFERING(0.5);

    double output = 0;
    State(double output){
        this.output = output;
    }

   }

   public void setRamp(double rampTime) {
     indexerMotor.getConfigurator().refresh(indexerConfig);
     indexerConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampTime;
     indexerMotor.getConfigurator().apply(indexerConfig);
   }

   public void configMotors() {
     indexerConfig = TalonConfigs.indexerConfigs();
     indexerMotor.getConfigurator().apply(indexerConfig);
   }

   public void setPercent(double Percentage) {
     mPeriodicIO.driveDemand = Percentage;
   }


   public Request stateRequest(State state){
    return new Request() {
        @Override
        public void act(){
            conformToState(state);
        }
    };
   }
   public void conformToState(State state){
    mPeriodicIO.driveDemand = state.output;
   }

   public Request hasPieceRequest(){
    return new Request() { 
        @Override 
        public boolean isFinished() {
            return mPeriodicIO.hasPiece;
        }
    };
   }
   public Request setPercentRequest(double percentage) {
     return new Request() {

       @Override
       public void act() {

         setPercent(percentage);
       }

     };

   }

   public boolean hasPiece(){
     return mPeriodicIO.hasPiece;
   }
   


   public double getStatorCurrent() {
     return mPeriodicIO.statorCurrent;
   }

   @Override
   public void writePeriodicOutputs() {
    
     mPeriodicIO.drivePosition = indexerMotor.getPosition().getValueAsDouble();
     mPeriodicIO.velocity = indexerMotor.getVelocity().getValueAsDouble();
     mPeriodicIO.statorCurrent = indexerMotor.getStatorCurrent().getValueAsDouble();
     mPeriodicIO.hasPiece = indexerBeamBreak.get();
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
     setPercentRequest(0);
   }

   public static class PeriodicIO {
     double drivePosition = 0;
     double velocity = 0;
     double statorCurrent = 0;

    boolean hasPiece = false;
     double rotationDemand = 0.0;
     double driveDemand = 0.0;
   }
 }
