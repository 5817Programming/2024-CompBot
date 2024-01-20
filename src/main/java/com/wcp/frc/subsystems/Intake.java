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

// public class Intake extends Subsystem {
//   private PeriodicIO mPeriodicIO = new PeriodicIO();
//   private TalonFX intakeMotor = new TalonFX(Ports.Intake);
//   private TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
//   private State currentState;
//   private boolean stateChanged;
//   private boolean driverHaptics;
//   private boolean hasPiece;
//   private double timeEnteredState = 0;

//   public static Intake instance = null;

//   public static Intake getInstance() {// if doesnt have an instance of swerve will make a new one
//     if (instance == null)
//       instance = new Intake();
//     return instance;
//   }

//   /** Creates a new intake. */
//   public Intake() {
//     configMotors();

//     currentState = State.Off;
//     stateChanged = false;
//     driverHaptics = false;
//     hasPiece = false;
//   }

//   public enum State {
//     Recieving(.5),
//     Intaking(1),
//     Feeding(1),
//     Holding(0),
//     Off(0);

//     double output = 0;

//     State(double output) {
//       this.output = output;
//     }
//   }

//   public void setRamp(double rampTime) {
//     intakeMotor.getConfigurator().refresh(intakeConfig);
//     intakeConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampTime;
//     intakeMotor.getConfigurator().apply(intakeConfig);
//   }

//   public void configMotors() {
//     intakeConfig = TalonDefaultConfig.driveConfigs();
//     intakeMotor.getConfigurator().apply(intakeConfig);
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
//       case Intaking:
//         if (stateChanged) {
//           setRamp(.5);
//           driverHaptics = false;
//           hasPiece = false;
//         }
//         if (getStatorCurrent() < 20) {
//           conformToState(State.Recieving);
//         }

//         break;
//       case Recieving:
//         if (stateChanged) {
//           driverHaptics = true;
//           hasPiece = true;
//         }
//         if (currentTime - timeEnteredState > .3) {
//           setState(State.Holding);
//         }
//         break;
//       case Off:
//         driverHaptics = false;
//         break;
//       case Feeding:
//         if (stateChanged) {
//           setRamp(0);
//           hasPiece = false;
//           driverHaptics = false;
//         }
//         if (currentTime - timeEnteredState > .3) {
//           stop();
//           setRamp(.5);
//           hasPiece = false;
//           driverHaptics = false;
//         }
//         break;
//       case Holding:
//         if(currentTime - timeEnteredState > 1) {
//           intakePercent(2/12);
//           System.out.println("Auto Ejecting Piece");
//           hasPiece = false;
//           driverHaptics = false;
//         }
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
//     mPeriodicIO.drivePosition = intakeMotor.getPosition().getValueAsDouble();
//     mPeriodicIO.velocity = intakeMotor.getVelocity().getValueAsDouble();
//     mPeriodicIO.statorCurrent = intakeMotor.getStatorCurrent().getValueAsDouble();
//   }

//   @Override
//   public void readPeriodicInputs() {
//     intakeMotor.setControl(new DutyCycleOut(mPeriodicIO.driveDemand, true, false, false, false));
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
