// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.wcp.frc.Constants;
import com.wcp.frc.Options;
import com.wcp.frc.Ports;
import com.wcp.frc.subsystems.encoders.Encoder;
import com.wcp.frc.subsystems.encoders.MagEncoder;
import com.wcp.lib.Conversions;
import com.wcp.lib.TalonDefaultConfig;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.util.Util;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Add your docs here. */
public class SwerveDriveModule extends Subsystem {
    FlywheelSim rotationSim;
    FlywheelSim driveSim;
    TalonFX rotationMotor, driveMotor;
    Encoder rotationMagEncoder;
    String name;
    protected int moduleID;
    double encoderOffset;

    private double previousEncDistance = 0;
	private Translation2d position = new Translation2d();
    private Translation2d mstartingPosition;

	private Pose2d estimatedRobotPose = new Pose2d();
	boolean standardCarpetDirection = true;
    
    Translation2d modulePosition;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.driveKS, Constants.driveKV,
            Constants.driveKA);

    boolean rotationEncoderFlipped;

    PeriodicIO mPeriodicIO = new PeriodicIO();

    /**
     * 
     * @param rotationMotorPort  -The Drive Motor Port
     * @param driveMotorPort     -The Drive Motor Port
     * @param moduleID           -The ID of the module
     * @param encoderStartingPos -The starting encoder position(used for zeroing
     *                           purposes)
     * @param modulePoseInches         -The position of the module relative to the robot's
     *                           center
     * @param flipEncoder        -Is the encoder going in the right direction?
     *                           (clockwise = increasing, counter-clockwise =
     *                           decreasing)
     */
    public SwerveDriveModule(int rotationMotorPort, int driveMotorPort, int moduleID, double encoderStartingPos,
            Translation2d modulePoseInches, boolean flipEncoder ,Translation2d moduleposemeters){
        this.rotationMotor = new TalonFX(rotationMotorPort);
        this.driveMotor = new TalonFX(driveMotorPort);
        this.driveSim = new FlywheelSim(DCMotor.getFalcon500(1), 7.63, .3);
        this.moduleID = moduleID;
        this.name = "Module " + moduleID;
        this.encoderOffset = encoderStartingPos;
        this.modulePosition = modulePoseInches;
        this.mstartingPosition =moduleposemeters;


        this.rotationEncoderFlipped = flipEncoder;

        if (Options.encoderType == "Mag Encoder") {
            rotationMagEncoder = new MagEncoder(Ports.SWERVE_ENCODERS[moduleID]);
        } 
        configMotors();
    }
    TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
    TalonFXConfiguration rotationConfigs = new TalonFXConfiguration();

    public void configMotors(){
        driveConfigs = TalonDefaultConfig.driveConfigs();
        rotationConfigs = TalonDefaultConfig.motionMagicConfig();

        driveMotor.getConfigurator().apply(driveConfigs);
        rotationMotor.getConfigurator().apply(rotationConfigs);

        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        rotationMotor.setNeutralMode(NeutralModeValue.Brake);


    }

    public enum ControlMode{
        PercentOuput,
        MotionMagic,
      }

    public void invertDriveMotor(boolean invert) {
        driveMotor.setInverted(invert);
    }

    public void invertRotationMotor(Boolean invertType) {
        rotationMotor.setInverted(invertType);
    }

    public void invertRotationMotor(boolean invert) {
        rotationMotor.setInverted(invert);
    }

    public void setDriveMotorNeutralMode(NeutralModeValue mode) {
        driveMotor.setNeutralMode(mode);
    }

    public void setModuleAngle(double desiredAngle) {
        // SmartDashboard.putNumber(this.name + " Module Commanded Angle", desiredAngle);// output telemetry
        desiredAngle = Util.placeInAppropriate0To360Scope(getModuleAngle(), desiredAngle);// optimization
        double angleRotations = degreesToRotations(desiredAngle);
        mPeriodicIO.rotationDemand = angleRotations;
    }

    public double getModuleAngle() {
        return rotationsToDegrees(mPeriodicIO.rotationPosition);
    }

    public double getModuleAbsolutePosition() {
        return rotationMagEncoder.getOutput() * ((this.rotationEncoderFlipped) ? -1 : 1) * 360.0;
    }

    public boolean isMagEncoderConnected() {
        return rotationMagEncoder.isConnected();
    }



    public void resetEncoders() {
        driveMotor.setPosition(0);

    }
    public synchronized void resetPose(Pose2d robotPose){
		Translation2d modulePosition = robotPose.transformBy(Pose2d.fromTranslation(mstartingPosition)).getTranslation();
		position = modulePosition;
	}

    public Pose2d getEstimatedRobotPose(){
		return estimatedRobotPose;
	}

    public Rotation2d getFieldCentricAngle(Rotation2d robotHeading){
		Rotation2d normalizedAngle =Rotation2d.fromDegrees( getModuleAngle());
		return normalizedAngle.rotateBy(robotHeading);
	}
	
    public double encUnitsToInches(double encUnits){
		return encUnits/Constants.kSwerveEncUnitsPerInch;
	}
    public SwerveModulePosition getPosition() {// returns swerve module postition
        return new SwerveModulePosition(
                Conversions.falconToMeters(driveMotor.getPosition().getValue(), Constants.kWheelCircumference, Options.driveRatio),
                Rotation2d.fromDegrees(getModuleAngle()));

    }
    
    public void setDriveOpenLoop(double percentOuput) {
        mPeriodicIO.driveControlMode = ControlMode.PercentOuput;
        mPeriodicIO.driveDemand = percentOuput;
    }
    
    public synchronized void updatePose(Rotation2d robotHeading){
		double currentEncDistance = Conversions.falconToMeters(driveMotor.getPosition().getValueAsDouble(), Constants.kWheelCircumference, Options.driveRatio);
		double deltaEncDistance = (currentEncDistance - previousEncDistance) * Constants.kWheelScrubFactors[moduleID];
		Rotation2d currentWheelAngle = getFieldCentricAngle(robotHeading);
		Translation2d deltaPosition = new Translation2d(-currentWheelAngle.cos()*deltaEncDistance, 
				currentWheelAngle.sin()*deltaEncDistance);


		double xScrubFactor = Constants.kXScrubFactor;
		double yScrubFactor = Constants.kYScrubFactor;
        if(Util.epsilonEquals(Math.signum(deltaPosition.getX()), 1.0)){
            xScrubFactor = .85;

        }else{
            xScrubFactor = .85;
        }
        if(Util.epsilonEquals(Math.signum(deltaPosition.getY()), 1.0)){
            yScrubFactor = .875;
        }else{
            yScrubFactor = .875;
        }
	
		deltaPosition = new Translation2d(deltaPosition.getX() * xScrubFactor,
			deltaPosition.getY() * yScrubFactor);
        Logger.recordOutput("delta t" + moduleID, deltaPosition.getY());
		Translation2d updatedPosition = position.translateBy(deltaPosition);
		Pose2d staticWheelPose = new Pose2d(updatedPosition, robotHeading);
		Pose2d robotPose = staticWheelPose.transformBy(Pose2d.fromTranslation(mstartingPosition).inverse());
		position = updatedPosition;
        Logger.recordOutput("Pos " + moduleID,robotPose.toWPI());
		estimatedRobotPose =  robotPose;
		previousEncDistance = currentEncDistance;
	}

    public double degreesToRotations(double degrees){
        return (degrees/360)*Options.rotationRatio;
    }

    public double rotationsToDegrees(double rotations){
        return (rotations*360)/Options.rotationRatio;
    }



    

    public void resetModulePositionToAbsolute() {
                double offset = getModuleAbsolutePosition() - encoderOffset;
                rotationMotor.setPosition(degreesToRotations(offset));
                System.out.print(this.name + offset);
        
            

    }

    public enum ModuleStatus {
        OK, ABSOLUTE_ENCODER_ERROR, DRIVE_MOTOR_ERROR, ROTATION_MOTOR_ERROR;
    }

    private boolean rotationMotorError = false;
    private boolean driveMotorError = false;

    public ModuleStatus getModuleStatus() {
        if (!isMagEncoderConnected())
            return ModuleStatus.ABSOLUTE_ENCODER_ERROR;
        else if (driveMotorError)
            return ModuleStatus.DRIVE_MOTOR_ERROR;
        else if (rotationMotorError)
            return ModuleStatus.ROTATION_MOTOR_ERROR;
        return ModuleStatus.OK;
    }

    @Override
    public void writePeriodicOutputs() {// updates data
        mPeriodicIO.rotationPosition = rotationMotor.getPosition().getValue();
        mPeriodicIO.drivePosition = driveMotor.getPosition().getValue();
        mPeriodicIO.velocity = driveMotor.getVelocity().getValue();
    }

      @Override
  public void readPeriodicInputs() {
    switch (mPeriodicIO.driveControlMode) {
      case PercentOuput:
          runPercentOutput(mPeriodicIO.driveDemand, driveMotor);
        break;
      case MotionMagic:
          runMotionMagic(mPeriodicIO.driveDemand, driveMotor);
    }
    switch (mPeriodicIO.rotationControlMode) {
      case PercentOuput:
          runPercentOutput(mPeriodicIO.rotationDemand, rotationMotor);
        break;
      case MotionMagic:
          runMotionMagic(mPeriodicIO.rotationDemand, rotationMotor);
    }
  }

  public void runPercentOutput(double percent, TalonFX motor){
    motor.setControl(new DutyCycleOut(percent, true, false, false, false));
  }
    public void runMotionMagic(double position, TalonFX motor){
    motor.setControl(new MotionMagicVoltage(position));
  }
    @Override
    public void outputTelemetry() {
        Logger.recordOutput(this.name+" Angle Demand", rotationsToDegrees(mPeriodicIO.rotationDemand));
        Logger.recordOutput(this.name + " Angle", rotationsToDegrees(mPeriodicIO.rotationPosition));
        Logger.recordOutput(this.name + " Mag Encoder Raw Value", getModuleAbsolutePosition() / 360.0);
        Logger.recordOutput(this.name + " Absolute Position", getModuleAbsolutePosition());
        Logger.recordOutput(this.name + " Drive Motor Demand", mPeriodicIO.driveDemand);
        Logger.recordOutput(this.name + " Status", getModuleStatus().toString());
        Logger.recordOutput(this.name + " Drive Position", mPeriodicIO.drivePosition);
        Logger.recordOutput(this.name + " A", Constants.kWheelCircumference);
        Logger.recordOutput(this.name + "Drive Control Mode", mPeriodicIO.driveControlMode);
        Logger.recordOutput(this.name + "Rotation Control Mode", mPeriodicIO.rotationControlMode);
    


    }

    @Override
    public void stop() {// stops everything
        setModuleAngle(getModuleAngle());
        setDriveOpenLoop(0.0);
    }

    public static class PeriodicIO {// data
        double rotationPosition = 0;
        double drivePosition = 0;
        double velocity = 0;

        ControlMode rotationControlMode = ControlMode.MotionMagic;
        ControlMode driveControlMode = ControlMode.PercentOuput;
        double rotationDemand = 0.0;
        double driveDemand = 0.0;
    }

}
