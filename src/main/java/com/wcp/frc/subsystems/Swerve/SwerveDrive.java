// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems.Swerve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.swing.text.html.Option;

import org.littletonrobotics.junction.Logger;

import com.wcp.frc.Constants;
import com.wcp.frc.Options;
import com.wcp.frc.Ports;
import com.wcp.frc.Planners.AutoAlignMotionPlanner;
import com.wcp.frc.Planners.DriveMotionPlanner;
import com.wcp.frc.subsystems.RobotState;
import com.wcp.frc.subsystems.RobotStateEstimator;
import com.wcp.frc.subsystems.Subsystem;
import com.wcp.frc.subsystems.Requests.Request;
import com.wcp.frc.subsystems.Vision.LimeLight;
import com.wcp.frc.subsystems.gyros.Pigeon;
import com.wcp.lib.Conversions;
import com.wcp.lib.HeadingController;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.swerve.ChassisSpeeds;
import com.wcp.lib.swerve.SwerveKinematics;
import com.wcp.lib.util.PID2d;
import com.wcp.lib.util.SynchronousPIDF;
import com.wcp.lib.util.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SwerveDrive extends Subsystem {
    public static SwerveDrive instance = null;

    public static SwerveDrive getInstance() {// if doesnt have an instance of swerve will make a new one
        if (instance == null)
            instance = new SwerveDrive();
        return instance;
    }

    SwerveDriveModule frontRightModule, frontLeftModule, rearLeftModule, rearRightModule;
    List<SwerveDriveModule> modules;

    Translation2d aimingVector = new Translation2d();
    Translation2d translationVector = new Translation2d();
    public double rotationScalar = 0;
    double speedPercent = 1;
    double rotationalVel;


    Pigeon gyro;
    LimeLight vision;
    Logger logger;
    Pose2d drivingPose;

    Pose2d poseMeters = new Pose2d();
    
    List<SwerveDriveModule> positionModules;
    Translation2d currentVelocity = new Translation2d();

    List<Translation2d> moduleVectors;

    final double translationDeadband = 0.1;
    final double rotationDeadband = 0.1;
    private boolean robotCentric = false;
    double desiredRotationScalar;
    double distanceTraveled;

    double lastUpdateTimestamp = 0;


    PID2d VisionPID;
    SynchronousPIDF areaVisionPID;

    private double lastTimestamp = Timer.getFPGATimestamp();
    SwerveKinematics inverseKinematics = new SwerveKinematics();
    RobotStateEstimator robotStateEstimator;
    AutoAlignMotionPlanner mAutoAlignMotionPlanner;
    DriveMotionPlanner driveMotionPlanner;
    RobotState robotState;
    public HeadingController headingController = new HeadingController();


    double currentSpeed = 0;
    double bestDistance;

    double lastTimeStamp = 0;

    public SwerveDrive() {

        frontRightModule = new SwerveDriveModule(Ports.FRONT_RIGHT_ROTATION, Ports.FRONT_RIGHT_DRIVE, 0,
                Constants.kFrontRightStartingEncoderPosition, Constants.kFrontRightPosition, true,
                Constants.mFrontRightPosition);
        frontLeftModule = new SwerveDriveModule(Ports.FRONT_LEFT_ROTATION, Ports.FRONT_LEFT_DRIVE, 1,
                Constants.kFrontLeftStartingEncoderPosition, Constants.kFrontLeftPosition, true,
                Constants.mFrontLeftPosition);
        rearLeftModule = new SwerveDriveModule(Ports.REAR_LEFT_ROTATION, Ports.REAR_LEFT_DRIVE, 2,
                Constants.kRearLeftStartingEncoderPosition, Constants.kRearLeftPosition, true,
                Constants.mRearLeftPosition);
        rearRightModule = new SwerveDriveModule(Ports.REAR_RIGHT_ROTATION, Ports.REAR_RIGHT_DRIVE, 3,
                Constants.kRearRightStartingEncoderPosition, Constants.kRearRightPosition, true,
                Constants.mRearRightPosition);
        modules = Arrays.asList(frontRightModule, frontLeftModule, rearLeftModule, rearRightModule);

        // sets which ways the modules turn
        frontRightModule.invertDriveMotor(true);
        frontLeftModule.invertDriveMotor(false);
        rearLeftModule.invertDriveMotor(false);
        rearRightModule.invertDriveMotor(true);
        positionModules = Arrays.asList(frontRightModule, frontLeftModule, rearLeftModule, rearRightModule);
        distanceTraveled = 0;

        gyro = Pigeon.getInstance();
        vision = LimeLight.getInstance();
        robotStateEstimator = RobotStateEstimator.getInstance();
        robotState = RobotState.getInstance();
        mAutoAlignMotionPlanner = new AutoAlignMotionPlanner();
        driveMotionPlanner =DriveMotionPlanner.getInstance();
    }



    public enum State {
        MANUAL,
        TRAJECTORY,
        OFF,
        AIMING,
        ALIGNMENT,
        SNAP,
    }

    private State currentState = State.MANUAL;

    public State getState() {
        return this.currentState;
    }

    public void setState(State desiredState) {
        currentState = desiredState;
    }

    public SwerveKinematics getKinematics(){
        return inverseKinematics;
    }



    public void setSpeedPercent(double percent) {
        speedPercent = (1 - (percent * Constants.GrannyModeWeight));
    }

    //
    public void sendInput(double x, double y, double rotation) {
        setState(State.MANUAL);
        translationVector = new Translation2d(x, y).scale(speedPercent);
        if (Math.abs(rotation) <= rotationDeadband) {
            rotation = 0;
        }
        if (rotation == 0 && rotationScalar != 0) {
            headingController.disableHeadingController(true);
        }
        rotationScalar = rotation;
        final double scaleValue = 1.5;
        double inputMagnitude = translationVector.norm();
        inputMagnitude = Math.pow(inputMagnitude, scaleValue);
        inputMagnitude = Util.deadband(translationDeadband, inputMagnitude);
        if (translationVector.norm() <= translationDeadband) {
            translationVector = new Translation2d();
        }
        rotationScalar *= 0.01;
        if (translationVector.norm() <= translationDeadband && Math.abs(rotation) <= rotationDeadband) {
            this.commandModuleDrivePowers(0);
        } else {
            this.update();

        }
        this.update();

    }



    public void parkMode() {
        rotationScalar = .5;
        rotationScalar *= 0.01;
        this.update();
        this.commandModuleDrivePowers(0);
    }

    public void commandModules(List<Translation2d> moduleVectors) {
        this.moduleVectors = moduleVectors;
        for (int i = 0; i < moduleVectors.size(); i++) {
            if (Util.shouldReverse(moduleVectors.get(i).direction(),
                    Rotation2d.fromDegrees(modules.get(i).getModuleAngle()))) {
                modules.get(i).setModuleAngle(moduleVectors.get(i).direction().getDegrees() + 180);
                modules.get(i).setDriveOpenLoop(-moduleVectors.get(i).norm());
            } else {
                modules.get(i).setModuleAngle(moduleVectors.get(i).direction().getDegrees());
                modules.get(i).setDriveOpenLoop(moduleVectors.get(i).norm());

            }
        }
    }

    public void commandModuleVelocitys(List<Translation2d> moduleVectors){
         this.moduleVectors = moduleVectors;
        for (int i = 0; i < moduleVectors.size(); i++) {
            if (Util.shouldReverse(moduleVectors.get(i).direction(),
                    Rotation2d.fromDegrees(modules.get(i).getModuleAngle()))) {
                modules.get(i).setModuleAngle(moduleVectors.get(i).direction().getDegrees() + 180);
                modules.get(i).setDriveOpenLoop(-moduleVectors.get(i).norm());
            } else {
                modules.get(i).setModuleAngle(moduleVectors.get(i).direction().getDegrees());
                modules.get(i).setDriveOpenLoop(moduleVectors.get(i).norm());

            }
        }
    }

    public void commandModuleDrivePowers(double power) {
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).setDriveOpenLoop(power);
        }
    }

    public void resetPose(Pose2d newPose) {
        modules.forEach((m) -> m.resetPose(newPose));
    }



    public void resetOdometry(Pose2d newpose, Rotation2d rotation) {
        Pose2d newpose2 = new Pose2d(newpose.getTranslation(), rotation);
        gyro.setAngle(rotation.getDegrees());
        modules.forEach((m) -> m.resetPose(newpose2));

    }




    public void zeroModules() {
        modules.forEach((m) -> {
            m.resetModulePositionToAbsolute();
        });
    }

    public Rotation2d getRobotHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public List<SwerveDriveModule> getModules(){
        return modules;
    }


    @Override
    public void update() {
        double timeStamp = Timer.getFPGATimestamp();
        poseMeters = robotState.getLatestOdomToVehicle().getValue();
        drivingpose = Pose2d.fromRotation(getRobotHeading());
        switch (currentState) {
            case MANUAL:
                double rotationCorrection = headingController.updateRotationCorrection(drivingpose.getRotation(),
                        timeStamp);
                if (translationVector.norm() == 0 || rotationScalar != 0) {
                    rotationCorrection = 0;
                }
                SmartDashboard.putNumber("Swerve Heading Correctiomm    33  33   /n", rotationCorrection);
                commandModules(inverseKinematics.updateDriveVectors(translationVector,
                        rotationScalar + rotationCorrection, drivingpose, robotCentric));
                break;

            case ALIGNMENT:
                ChassisSpeeds targetChassisSpeeds = updateAutoAlign();
                commandModuleVelocitys(inverseKinematics.updateDriveVectors(new Translation2d(
                    targetChassisSpeeds.vxMetersPerSecond,
                    targetChassisSpeeds.vyMetersPerSecond),
                    targetChassisSpeeds.omegaRadiansPerSecond,
                    poseMeters,
                    robotCentric
                    ));
                break;

            case TRAJECTORY:
                driveMotionPlanner.updateTrajectory();
                Translation2d translationCorrection = driveMotionPlanner.updateFollowedTranslation2d(timeStamp).scale(1);
                headingController.setTargetHeading(driveMotionPlanner.getTargetHeading());
                rotationCorrection = headingController.getRotationCorrection(getRobotHeading(), timeStamp);
                desiredRotationScalar = rotationCorrection;
                commandModules(inverseKinematics.updateDriveVectors(translationCorrection, rotationCorrection, poseMeters,
                        robotCentric));
                break;

            case AIMING:
                commandModules(
                        inverseKinematics.updateDriveVectors(aimingVector, rotationScalar, drivingpose, robotCentric));
                break;

            case OFF:
                commandModules(inverseKinematics.updateDriveVectors(new Translation2d(), 0, drivingpose, robotCentric));
                break;

            case SNAP:
                headingController.setTargetHeading(driveMotionPlanner.getTargetHeading());
                rotationCorrection = headingController.getRotationCorrection(getRobotHeading(), timeStamp);
                desiredRotationScalar = rotationCorrection;
                commandModules(inverseKinematics.updateDriveVectors(translationVector, rotationCorrection, poseMeters,
                        robotCentric));
                break;

        }

    }

    public ChassisSpeeds updateAutoAlign(){
        final double now = Timer.getFPGATimestamp();
        var fieldToOdometry = robotState.getFieldToOdom(now);
        var odomToVehicle = robotState.getOdomToVehicle(now);
        ChassisSpeeds output = mAutoAlignMotionPlanner.updateAutoAlign(now, odomToVehicle, Pose2d.fromTranslation(fieldToOdometry), robotState.getMeasuredVelocity());
        return output;
    }

    public void snap(double r) {
        setState(State.SNAP);
        headingController.setTargetHeading(Rotation2d.fromDegrees(r));
    }

    public void updateOdometry(double timestamp) {// uses sent input to commad modules and correct for rotatinol drift

        lastUpdateTimestamp = timestamp;

    }

    public Pose2d getPoseMeters() {
        return poseMeters;
    }

    public double getRotationalVelSIM() {

        return desiredRotationScalar;
    }

    public void fieldzeroSwerve() {// starts the zero 180 off
        headingController.setTargetHeading(Rotation2d.fromDegrees(-180));
        gyro.setAngle(-180);
    }

    public Translation2d getVelocity() {
        return currentVelocity;
    }

    @Override
    public void readPeriodicInputs() {
        modules.forEach((m) -> {
            m.readPeriodicInputs();
        });
    }

    @Override
    public void writePeriodicOutputs() {
        modules.forEach((m) -> {
            m.writePeriodicOutputs();
        });
    }

    public void resetGryo(double angle) {
        headingController.setTargetHeading(Rotation2d.fromDegrees(angle));
        gyro.setAngle(angle);
    }

    public void zeroSwerve() {// zeros gyro
        headingController.setTargetHeading(Rotation2d.fromDegrees(0));
        gyro.setAngle(0);
        resetPose(new Pose2d(new Translation2d(1.9, 4.52), Rotation2d.fromDegrees(0)));
    }

    public void resetEncoders() {// zeros encoders
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).resetEncoders();
        }

    }

    public Request setStateRequest(State state) {
        return new Request() {
            @Override
            public void act() {
                setState(state);
            }
        };
    }

    public synchronized ChassisSpeeds getSetPoint(){
        var desiredThrottleSpeed = translationVector.x() * Constants.SwerveMaxspeedMPS;
        var desiredStrafeSpeed = translationVector.y() * Constants.SwerveMaxspeedMPS;
        var desiredRotationSpeed = rotationScalar * Conversions.falconToMPS(Constants.kSwerveRotationMaxSpeed, Constants.kWheelCircumference, Options.driveRatio);
        return ChassisSpeeds.fromRobotRelativeSpeeds(desiredThrottleSpeed, desiredStrafeSpeed, desiredRotationSpeed);
    }


    public Pose2d targetApril() {
        double xError = 0;
        double yError = 0;

        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastTimeStamp;
        if (vision.hasTarget()) {
            VisionPID.x().setOutputRange(-.2, .2);
            VisionPID.y().setOutputRange(-.2, .2);

            xError = VisionPID.x().calculate(vision.getX(), dt);
            yError = VisionPID.y().calculate(0 - .05, dt); //TODO REIMPLEMENTVISION
        }

        else {
            xError = 0;
            yError = 0;
        }
        lastTimeStamp = currentTime;
        return new Pose2d(new Translation2d(yError, xError).inverse(), Rotation2d.fromDegrees(180));
    }

    public void snapToPoint(Pose2d targetPoint){
        if(mAutoAlignMotionPlanner != null){
            if(currentState != State.ALIGNMENT){
                mAutoAlignMotionPlanner.reset();
                setState(State.ALIGNMENT);
            }
        }
        mAutoAlignMotionPlanner.setTargetPoint(targetPoint);
        robotState.setDisplaySetpointPose(targetPoint);
    }

    public Pose2d targetObject(boolean fixedRotation) {
        double xError = 0;
        double yError = 0;
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastTimeStamp;
        if (vision.hasTarget()) {
            VisionPID.x().setOutputRange(-.2, .2);
            areaVisionPID.setOutputRange(-.2, .2);
            VisionPID.x().setOutputMagnitude(.03);
            areaVisionPID.setOutputMagnitude(.02);

            xError = VisionPID.x().calculate(vision.getX(), dt);
            yError = areaVisionPID.calculate(vision.getArea() - 20, dt);
        }
        else {
            xError = 0;
            yError = 0;
        }
        lastTimeStamp = currentTime;
        return new Pose2d(new Translation2d(yError, -xError).inverse(),
                (fixedRotation ? new Rotation2d() : getRobotHeading()));
    }

    public Request openLoopRequest(Translation2d x, double r) {
        return new Request() {

            @Override
            public void act() {
                setState(State.MANUAL);
                sendInput(x.x(), x.y(), r);

            }

        };

    }










    @Override
    public void outputTelemetry() {
        Logger.recordOutput("Odometry", poseMeters.toWPI());
        modules.forEach((m) -> {
            m.outputTelemetry();
        });
    }

    @Override
    public void stop() {// stops everything
        setState(State.MANUAL);
        translationVector = new Translation2d();
        rotationScalar = 0;

        update();
        commandModuleDrivePowers(0);

    }

}
