// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.wcp.frc.Constants;
import com.wcp.frc.Ports;
import com.wcp.frc.subsystems.Requests.Request;
import com.wcp.frc.subsystems.gyros.Pigeon;
import com.wcp.lib.HeadingController;
import com.wcp.lib.SwerveInverseKinematics;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.util.PID2d;
import com.wcp.lib.util.PathFollower;
import com.wcp.lib.util.SynchronousPIDF;
import com.wcp.lib.util.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Swerve extends Subsystem {
    public static Swerve instance = null;

    public static Swerve getInstance() {// if doesnt have an instance of swerve will make a new one
        if (instance == null)
            instance = new Swerve();
        return instance;
    }

    SwerveDriveModule frontRightModule, frontLeftModule, rearLeftModule, rearRightModule;
    List<SwerveDriveModule> modules;

    Translation2d aimingVector = new Translation2d();
    Translation2d translationVector = new Translation2d();
    public double rotationScalar = 0;
    double speedPercent = 1;
    double rotationalVel;
    boolean trajectoryStarted = false;
    boolean trajectoryFinished = false;
    double speed;
    boolean useAllianceColor;

    Pigeon gyro;
    Vision vision;
    PathFollower pathFollower;
    Logger logger;

    Pose2d pose = new Pose2d();
    Pose2d drivingpose = new Pose2d();
    PathPlannerTrajectory trajectoryDesired;
    List<SwerveDriveModule> positionModules;
    Translation2d currentVelocity = new Translation2d();

    List<Translation2d> moduleVectors;

    final double translationDeadband = 0.1;
    final double rotationDeadband = 0.1;
    private boolean robotCentric = false;
    double desiredRotationScalar;
    double distanceTraveled;

    double lastUpdateTimestamp = 0;

    private Translation2d targetFollowTranslation = new Translation2d();
    private Rotation2d targetHeading = new Rotation2d();


    PID2d OdometryPID;
    PID2d VisionPID;
    SynchronousPIDF areaVisionPID;

    private double lastTimestamp = Timer.getFPGATimestamp();
    SwerveInverseKinematics inverseKinematics = new SwerveInverseKinematics();
    public HeadingController headingController = new HeadingController();

    boolean pathStarted;
    PIDController thetaController;
    PIDController advanceController;

    SynchronousPIDF rPID;
    double currentSpeed = 0;
    double bestDistance;
    
    double lastTimeStamp = 0;
    public Swerve() {

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

        pathFollower = PathFollower.getInstance();
        gyro = Pigeon.getInstance();
        vision = Vision.getInstance();
        logger = Logger.getInstance();

        OdometryPID = new PID2d(new SynchronousPIDF(1, 0.0, 0),
                                new SynchronousPIDF(1, 0.0, 0));

        VisionPID = new PID2d(new SynchronousPIDF(.01, 0.005, 0.01),
                              new SynchronousPIDF(.5, 0.01, 0));

        areaVisionPID = new SynchronousPIDF(.025, 0.01, 0);

    }

    public void setTrajectory(PathPlannerTrajectory trajectory, double nodes) {
        trajectoryFinished = false;
        pathFollower.setTrajectory(trajectory, nodes);
        pose = pathFollower.getInitial(trajectory);
        Pose2d newpose = (pathFollower.getInitial(trajectory));
        modules.forEach((m) -> m.resetPose(new Pose2d(newpose.getTranslation(), new Rotation2d())));
        gyro.setAngle(newpose.getRotation().getDegrees());

    }

    public enum State {
        MANUAL,
        TRAJECTORY,
        OFF,
        AIMING,
        AMP,
        SNAP,
    }

    private State currentState = State.MANUAL;

    public State getState() {
        return this.currentState;
    }

    public void setState(State desiredState) {
        currentState = desiredState;
    }

    public void startPath( boolean useAllianceColor) {
        trajectoryStarted = true;
        this.useAllianceColor = useAllianceColor;
        pathFollower.startTimer();

    }

    public void setSpeedPercent(double percent){
        speedPercent = (1-(percent*Constants.GrannyModeWeight));
    }

    //
    public void sendInput(double x, double y, double rotation) {
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

    public void updateTrajectory() {
        Pose2d desiredPose = pathFollower.getDesiredPose2d(useAllianceColor, getPose());
        targetHeading = desiredPose.getRotation().inverse();
        targetFollowTranslation = desiredPose.getTranslation();
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

    public void commandModuleDrivePowers(double power) {
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).setDriveOpenLoop(power);
        }
    }

    public void resetPose(Pose2d newPose) {
        modules.forEach((m) -> m.resetPose(newPose));
    }

    /** The tried and true algorithm for keeping track of position */
    public synchronized void updatePose(double timestamp) {

        double x = 0.0;
        double y = 0.0;
        Rotation2d heading = getRobotHeading();

        double averageDistance = 0.0;
        double[] distances = new double[4];
        for (SwerveDriveModule m : positionModules) {
            m.updatePose(heading);
            double distance = m.getEstimatedRobotPose().getTranslation().translateBy(pose.getTranslation().inverse())
                    .norm();
            distances[m.moduleID] = distance;
            averageDistance += distance;
        }
        averageDistance /= positionModules.size();

        int minDevianceIndex = 0;
        double minDeviance = Units.inchesToMeters(100);
        List<SwerveDriveModule> modulesToUse = new ArrayList<>();
        for (SwerveDriveModule m : positionModules) {
            double deviance = Math.abs(distances[m.moduleID] - averageDistance);
            if (deviance < minDeviance) {
                minDeviance = deviance;
                minDevianceIndex = m.moduleID;
            }
            if (deviance <= 10000) {
                modulesToUse.add(m);
            }
        }

        if (modulesToUse.isEmpty()) {
            modulesToUse.add(modules.get(minDevianceIndex));
        }

        // SmartDashboard.putNumber("Modules Used", modulesToUse.size());

        for (SwerveDriveModule m : modulesToUse) {
            x += m.getEstimatedRobotPose().getTranslation().getX();
            y += m.getEstimatedRobotPose().getTranslation().getY();
        }
        Pose2d updatedPose = new Pose2d(new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()), heading);
        Translation2d deltaPos = updatedPose.getTranslation().translateBy(pose.getTranslation().inverse());
        distanceTraveled += deltaPos.getNorm();
        currentSpeed = deltaPos.getNorm() / (timestamp - lastUpdateTimestamp);
        currentVelocity = deltaPos.scale(1/(timestamp - lastUpdateTimestamp));

        pose = updatedPose;

        modules.forEach((m) -> m.resetPose(pose));
        lastUpdateTimestamp = timestamp;
    }

    public void resetOdometry(Pose2d newpose, Rotation2d rotation) {
        Pose2d newpose2 = new Pose2d(newpose.getTranslation(), rotation);
        modules.forEach((m) -> m.resetPose(newpose2));

    }

    public void resetTimer() {
        PathFollower.getInstance().resetTimer();
    }
    

    public Translation2d updateFollowedTranslation2d(double timestamp) {
        double dt = timestamp - lastTimestamp;
        Translation2d currentRobotPositionFromStart = pose.getTranslation();
        OdometryPID.x().setOutputRange(-.9, .9);
        OdometryPID.y().setOutputRange(-.9, .9);
        double xError = OdometryPID.x().calculate(targetFollowTranslation.getX() - currentRobotPositionFromStart.getX(), dt);
        double yError = OdometryPID.y().calculate(targetFollowTranslation.getY() - currentRobotPositionFromStart.getY(), dt);
        lastTimestamp = timestamp;
        if (Math.abs(xError + yError) / 2 < .1 && PathFollower.getInstance().isFinished()) {
            setState(State.OFF);
            trajectoryFinished = true;
            return new Translation2d();
        }
        return new Translation2d(xError, -yError);
    }

    public void zeroModules() {
        modules.forEach((m) -> {
            m.resetModulePositionToAbsolute();
        });
    }

    public Rotation2d getRobotHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    @Override
    public void update() {
        updatePose(Timer.getFPGATimestamp());
        double timeStamp = Timer.getFPGATimestamp();
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

            case AMP:
                headingController.setTargetHeading(targetHeading);
                rotationCorrection = headingController.getRotationCorrection(getRobotHeading(), timeStamp);
                SmartDashboard.putNumber("Swerve Heading Correctiomm    33  33   /n", rotationCorrection);
                commandModules(inverseKinematics.updateDriveVectors(translationVector.translateBy(aimingVector),
                        rotationCorrection, drivingpose,
                        robotCentric));
                break;

            case TRAJECTORY:
                System.out.println("hwlakhfa");
                updateTrajectory();
                Translation2d translationCorrection = updateFollowedTranslation2d(timeStamp).scale(1);
                headingController.setTargetHeading(targetHeading);
                rotationCorrection = headingController.getRotationCorrection(getRobotHeading(), timeStamp);
                desiredRotationScalar = rotationCorrection;
                commandModules(inverseKinematics.updateDriveVectors(translationCorrection, rotationCorrection, pose,
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
                headingController.setTargetHeading(targetHeading);
                rotationCorrection = headingController.getRotationCorrection(getRobotHeading(), timeStamp);
                desiredRotationScalar = rotationCorrection;
                commandModules(inverseKinematics.updateDriveVectors(translationVector, rotationCorrection, pose,
                        robotCentric));
                break;

        }

    }

    public void snap(double r) {
        setState(State.SNAP);
        targetHeading = Rotation2d.fromDegrees(r);
        headingController.setTargetHeading(targetHeading);
    }

    public void Aim(Translation2d aimingVector, double scalar) {
        currentState = State.AIMING;
        this.aimingVector = aimingVector;
        this.rotationScalar = scalar;
        update();
    }
    
    public void Aim(Pose2d aimingVector) {
        currentState = State.AMP;
        this.aimingVector = aimingVector.getTranslation();
        targetHeading = aimingVector.getRotation();
        headingController.setTargetHeading(targetHeading);
        update();
    }

    public void Aim(Translation2d aimingVector, Rotation2d rotation) {
        currentState = State.AMP;// SETS HEADING TO 0or 180
        this.aimingVector = aimingVector;
        targetHeading = rotation;
        headingController.setTargetHeading(rotation);
        update();

    }

    public void updateOdometry(double timestamp) {// uses sent input to commad modules and correct for rotatinol drift

        lastUpdateTimestamp = timestamp;

    }

    public Pose2d getPose() {
        return pose;
    }

    public double getRotationalVelSIM() {

        return desiredRotationScalar;
    }

    public void fieldzeroSwerve() {// starts the zero 180 off
        headingController.setTargetHeading(Rotation2d.fromDegrees(-180));
        gyro.setAngle(-180);
    }

    


    public Translation2d getVelocity(){
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

    public Request objectTargetRequest(boolean fixedRotation) {
        return new Request() {

            @Override
            public void act() {
                setState(State.AMP);
                Aim(targetObject(fixedRotation));
            }

            @Override
            public void initialize() {
                vision.setPipeline(0);
            }

            @Override
            public boolean isFinished() {
                Translation2d translation = targetObject(fixedRotation).getTranslation();
                if(translation.within(.3) && vision.hasTarget()){
                    aimingVector = new Translation2d();
                }
                return translation.within(.3) && vision.hasTarget();
            }

        };
    }

    public Request aimStateRequest(boolean cube) {
        return new Request() {

            public void act() {
                setState(State.AMP);
                Aim(targetApril());
            }

            @Override
            public void initialize() {
                vision.setPipeline(1);
            }

            @Override
            public boolean isFinished() {
                Translation2d translation = targetApril().getTranslation();
                if(translation.within(.4) && vision.hasTarget()){
                    aimingVector = new Translation2d();
                }
                if(translation.within(.4) && vision.hasTarget()){
                    System.out.println("Done. "); 
                }
                return translation.within(.4) && vision.hasTarget();
            }
        };

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
            yError = VisionPID.y().calculate(vision.getDistance()-.05, dt);
        }

        else {
            xError = 0;
            yError = 0;
        }
        lastTimeStamp = currentTime;
        return new Pose2d(new Translation2d(yError, xError).inverse(), Rotation2d.fromDegrees(180));
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
            areaVisionPID.setOutputMagnitude(.02
            );

            xError = VisionPID.x().calculate(vision.getX(), dt);
            yError = areaVisionPID.calculate(vision.getArea() - 20, dt);
        }

        else {
            xError = 0;
            yError = 0;
        }
        lastTimeStamp = currentTime;
        return new Pose2d(new Translation2d(yError, -xError).inverse(), (fixedRotation ? new Rotation2d() : getRobotHeading()));
    }

    public Request openLoopRequest(Translation2d x, double r) {
        return new Request() {

            @Override
            public void act() {
                setState(State.MANUAL);
                sendInput(x.getX(), x.getY(), r);

            }

        };

    }
    
    public Request startPathRequest(boolean useAllianceColor) {
        return new Request() {
            @Override
            public void act() {
                setState(State.TRAJECTORY);
                startPath(useAllianceColor);
            }

            @Override
            public boolean isFinished(){
                return false;
            }
        };
    }

    public Request waitForTrajectoryRequest(double PercentageToRun) {
        return new Request() {
            @Override
            public boolean isFinished() {
                return pathFollower.hasElapsedPercentage(PercentageToRun);
            }
        };
    }

    public Request waitForTrajectoryRequest() {
        return new Request() {
            @Override
            public boolean isFinished() {
                return trajectoryFinished;
            }
        };
    }

    // public Request generateTrajectoryRequest(int node) {
    //     return new Request() {

    //         @Override
    //         public void act() {
    //             PathPlannerTrajectory trajectory = PathGenerator.generatePath(new PathConstraints(4, 4),
    //                     new Node(Constants.scoresY.get(node), DriverStation.getAlliance() == Alliance.Blue ? 2 : 14.71),
    //                     Constants.FieldConstants.obstacles);
    //             setTrajectory(trajectory);TODO
    //         }

    //     };

    // }

    // public Request generateTrajectoryRequest(Node node) {
    //     return new Request() {

    //         @Override
    //         public void act() {
    //             PathPlannerTrajectory trajectory = PathGenerator.generatePath(new PathConstraints(4, 4), node,
    //                     Constants.FieldConstants.obstacles);
    //             setTrajectory(trajectory);
    //         }

    //     };

    // }

    public Request setTrajectoryRequest(PathPlannerTrajectory trajectory, double nodes) {
        return new Request() {

            @Override
            public void act() {
                setTrajectory(trajectory, nodes);
            }

        };
    }

    @Override
    public void outputTelemetry() {
        logger.recordOutput("Odometry", pose.toWPI());        modules.forEach((m) -> {
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
