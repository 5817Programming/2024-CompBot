// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package com.wcp.frc.subsystems;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.wcp.frc.Ports;
// import com.wcp.frc.subsystems.Requests.Request;
// import com.wcp.lib.TalonDefaultConfig;

// import edu.wpi.first.wpilibj.Timer;

// public class Indexer extends Subsystem {
//   private PeriodicIO mPeriodicIO = new PeriodicIO();
//   private TalonFX indexerMotor = new TalonFX(Ports.Intake);
//   private TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
//   private State currentState;
//   private boolean stateChanged;
//   private boolean hasPiece;
//   private double timeEnteredState = 0;

//   public static Indexer instance = null;

//   public static Indexer getInstance() {// if doesnt have an instance of swerve will make a new one
//     if (instance == null)
//       instance = new Indexer();
//     return instance;
//   }

//   /** Creates a new intake. */
//   public Indexer() {
//     configMotors();

//     currentState = State.Off;
//     stateChanged = false;
//     hasPiece = false;
//   }

//   public enum State {
//     Recieving(.5),
//     Feeding(1),
//     Holding(0),
//     Off(0);

//     double output = 0;

//     State(double output) {
//       this.output = output;
//     }
//   }

//   public void setRamp(double rampTime) {
//     indexerMotor.getConfigurator().refresh(intakeConfig);
//     intakeConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampTime;
//     indexerMotor.getConfigurator().apply(intakeConfig);
//   }

//   public void configMotors() {
//     intakeConfig = TalonDefaultConfig.driveConfigs();
//     indexerMotor.getConfigurator().apply(intakeConfig);
//   }

//   public void intakePercent(double Percentage) {
//     mPeriodicIO.driveDemand = Percentage;
//   }

//   public void setState(State state) {
//     if (state != currentState)
//       stateChanged = true;
//     currentState = state;
//     timeEnteredState = Timer.getFPGATimestamp();
//   }

//   public void conformToState(State state) {
//     setState(state);
//     setIntakePercentRequest(state.output);
//   }

//   public Request stateRequest(State state) {
//     return new Request() {

//       @Override
//       public void act() {
//         conformToState(state);
//       }
//     };
//   }

//   public Request setIntakePercentRequest(double percentage) {
//     return new Request() {

//       @Override
//       public void act() {

//         intakePercent(percentage);
//       }

//     };

//   }

//   @Override
//   public void update() {
//     double currentTime = Timer.getFPGATimestamp();
//     switch (currentState) {
//       case Recieving:
//         if (stateChanged) {
//           hasPiece = false;
//         }
//         if (currentTime - timeEnteredState > .3) {//TODO && BeamBreak = true;
//           setState(State.Holding);
//         }
//         break;
//       case Off:
//         setState(State.Off);
//         break;
//       case Feeding:
//         if (stateChanged) {
//           setRamp(0);
//           hasPiece = false;
//         }
//         if (currentTime - timeEnteredState > .3) {
//           stop();
//           setRamp(.5);
//           hasPiece = false;
//         }
//         break;
//       case Holding:
//         hasPiece = true;
//         break;

//     }

//   }

//   public Request hasPieceRequest(){
//     return new Request() {
//       @Override
//         public boolean isFinished() {
//             return !stateChanged && hasPiece;
//         }
//     };
//   }

//   public double getStatorCurrent() {
//     return mPeriodicIO.statorCurrent;
//   }

//   @Override
//   public void writePeriodicOutputs() {
//     mPeriodicIO.drivePosition = indexerMotor.getPosition().getValueAsDouble();
//     mPeriodicIO.velocity = indexerMotor.getVelocity().getValueAsDouble();
//     mPeriodicIO.statorCurrent = indexerMotor.getStatorCurrent().getValueAsDouble();
//   }

//   @Override
//   public void readPeriodicInputs() {
//     indexerMotor.setControl(new DutyCycleOut(mPeriodicIO.driveDemand, true, false, false, false));
//   }

//   @Override
//   public void outputTelemetry() {

//   }

//   @Override
//   public void stop() {
//     setIntakePercentRequest(0);
//   }

//   public static class PeriodicIO {// data
//     double drivePosition = 0;
//     double velocity = 0;
//     double statorCurrent = 0;

//     double rotationDemand = 0.0;
//     double driveDemand = 0.0;
//   }
// }
