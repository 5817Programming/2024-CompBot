// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.Planners;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.uni.frc.subsystems.RobotState;
import com.uni.frc.subsystems.RobotStateEstimator;
import com.uni.frc.subsystems.Requests.Request;
import com.uni.frc.subsystems.gyros.Pigeon;
import com.uni.lib.HeadingController;
import com.uni.lib.PathHeadingController;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Translation2d;
import com.uni.lib.motion.PathGenerator;
import com.uni.lib.motion.PathStateGenerator;
import com.uni.lib.swerve.ChassisSpeeds;
import com.uni.lib.util.LoggedTunableNumber;
import com.uni.lib.util.PID2d;
import com.uni.lib.util.SynchronousPIDF;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DriveMotionPlanner {

    boolean useAllianceColor;
    boolean noteTracked = false;
    private Translation2d targetFollowTranslation = new Translation2d();
    private Rotation2d targetHeading = new Rotation2d();

    private final LoggedTunableNumber autoP = new LoggedTunableNumber("Auto/P",8);
    private boolean trajectoryStarted = false;
    private boolean trajectoryFinished = false;
    private Pose2d drivingpose = new Pose2d();
    private Pose2d notePose = Pose2d.identity();
    private PathPlannerTrajectory trajectoryDesired;
    private PathStateGenerator mPathStateGenerator;
    private PathGenerator mPathGenerator;
    private PID2d OdometryPID = new PID2d(new SynchronousPIDF(autoP.get(), 0., 0.3), new SynchronousPIDF(autoP.get(), 0.0, 0.3));
    private PathHeadingController ThetaController = new PathHeadingController(0.003, 0.0, 0, 0.0);
    private SynchronousPIDF mTrackingPID = new SynchronousPIDF(.01 ,0 ,0.005); 
    private double lastTimestamp = 0;
    public static DriveMotionPlanner instance = null;
    State desiredState;

    public static DriveMotionPlanner getInstance() {

        if (instance == null)
            instance = new DriveMotionPlanner();
        return instance;
    }

    public DriveMotionPlanner() {
        mPathStateGenerator = PathStateGenerator.getInstance();
        mPathGenerator = new PathGenerator();
        // swerve = SwerveDrive.getInstance();
    }

    public Rotation2d getTargetHeading() {
        return targetHeading;
    }

    public void setTrajectory(PathPlannerTrajectory trajectory, double initRotation, boolean useAllianceColor) {
        trajectoryFinished = false;
        this.useAllianceColor = useAllianceColor;
        mPathStateGenerator.setTrajectory(trajectory);
        Pose2d newpose = (mPathStateGenerator.getInitial(trajectory, initRotation, useAllianceColor));
        Logger.recordOutput("Auto/initalPose", newpose.toWPI());
        for(int i = 0; i<3; i++)
            RobotStateEstimator.getInstance().resetOdometry(newpose);
        Pigeon.getInstance().setAngle(Rotation2d.fromDegrees(initRotation).getDegrees());
        if(DriverStation.getAlliance().get().equals(Alliance.Red))
            Pigeon.getInstance().setAngle(Rotation2d.fromDegrees(initRotation).flip().inverse().getDegrees());

    }

    public void resetNoteTracking(){
        noteTracked = false;
    }

    public void setTrajectoryOnTheFly(PathPlannerTrajectory trajectory, boolean useAllianceColor) {
        trajectoryFinished = false;
        this.useAllianceColor = useAllianceColor;
        mPathStateGenerator.setTrajectory(trajectory);

        Pose2d newpose = RobotState.getInstance().getKalmanPose(Timer.getFPGATimestamp());
        RobotStateEstimator.getInstance().resetModuleOdometry(newpose);
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), newpose);
    }

    public void startPath(boolean useAllianceColor) {
        trajectoryStarted = true;
        this.useAllianceColor = useAllianceColor;
        mPathStateGenerator.startTimer();
    }

    public void resetTimer() {
        PathStateGenerator.getInstance().resetTimer();
    }

    public Request setTrajectoryRequest(PathPlannerTrajectory trajectory, double initRotation,
            boolean useAllianceColor) {
        return new Request() {

            @Override
            public void act() {
                setTrajectory(trajectory, initRotation, useAllianceColor);
            }

        };
    }

    public void updateTrajectory() {
        desiredState = mPathStateGenerator.getDesiredState(useAllianceColor);
    }

    public Pose2d getTwist2dToFollow(double timestamp) {
        targetFollowTranslation = new Translation2d(desiredState.getTargetHolonomicPose().getTranslation());

        if(autoP.hasChanged(hashCode())){
            OdometryPID.x().setPID(autoP.get(), 0.0, OdometryPID.x().getD());
            OdometryPID.y().setPID(autoP.get(), 0.0, OdometryPID.y().getD());

        }

        double dt = timestamp - lastTimestamp;

        Rotation2d currentRotation = DriverStation .getAlliance().get().equals(Alliance.Red)?RobotState.getInstance().getLatestKalmanPose().getRotation().flip():RobotState.getInstance().getLatestKalmanPose().getRotation();
        Pose2d currentRobotPositionFromStart =new Pose2d(RobotState.getInstance().getLatestKalmanPose().getTranslation(), currentRotation );


        double xFF =
            desiredState.velocityMps * desiredState.heading.getCos();
        double yFF =
            desiredState.velocityMps * desiredState.heading.getSin();
        Rotation2d targetRotation = new Rotation2d(desiredState.targetHolonomicRotation).inverse();

        Logger.recordOutput("targetRotation", targetRotation.getRotation().getDegrees());
        ThetaController.setTargetHeading(targetRotation);
        double rotationFeedback = ThetaController.getRotationCorrection(currentRobotPositionFromStart.getRotation(), timestamp);
           

        Logger.recordOutput("Auto/Desired Pose", new Pose2d(targetFollowTranslation , new Rotation2d(desiredState.targetHolonomicRotation)).toWPI());
        double xError = OdometryPID.x().calculate(targetFollowTranslation.x() - currentRobotPositionFromStart.getTranslation().x(), dt) + xFF;
        double yError = OdometryPID.y().calculate(targetFollowTranslation.y() - currentRobotPositionFromStart.getTranslation().y(), dt) + yFF;
        targetHeading = new Rotation2d(desiredState.targetHolonomicRotation);
        lastTimestamp = timestamp;
        if (((Math.abs(xError) + Math.abs(yError)) / 2 < .05 && PathStateGenerator.getInstance().isFinished())
                || trajectoryFinished) {
            trajectoryFinished = true;
            return new Pose2d(0,0,Rotation2d.identity());
        }
        return new Pose2d(xError, -yError, Rotation2d.fromDegrees(-rotationFeedback));
    }

    public Pose2d getTranslation2dToTrack(double timestamp, Pose2d notePose) {
        double dt = timestamp - lastTimestamp;
        Translation2d currentRobotPositionFromStart = RobotState.getInstance().getLatestKalmanPose()
                .getTranslation();
        if(notePose.getTranslation().translateBy(currentRobotPositionFromStart).norm() > .3|| noteTracked)
            return new Pose2d(getTwist2dToFollow(timestamp).getTranslation(), getTargetHeading());
        OdometryPID.x().setOutputRange(-1, 1);
        OdometryPID.y().setOutputRange(-1, 1);
        Logger.recordOutput("Auto/Desired Pose", Pose2d.fromTranslation(targetFollowTranslation).toWPI());
        double xError = OdometryPID.x().calculate(notePose.getTranslation().x() - currentRobotPositionFromStart.x(), dt);
        double yError = OdometryPID.y().calculate(notePose.getTranslation().y() - currentRobotPositionFromStart.y(), dt);

        Rotation2d targetAngle = notePose.getTranslation().translateBy(currentRobotPositionFromStart.getTranslation().inverse()).getAngle().flip();
        lastTimestamp = timestamp;
        
        if ((Math.abs(xError) + Math.abs(yError)) / 2 < .05) {
            noteTracked = true;
        }
        return null;
        // return new Pose2d(xError, yError, targetAngle);
        }
    public void generateAndPrepareTrajectory(Pose2d startPose, Pose2d endPose, ChassisSpeeds currentVelocity,
            boolean useAllianceColor) {
        PathPlannerTrajectory trajectory = mPathGenerator.generatePath(
                startPose,
                new PathConstraints(2.5, 7, 1000000, 1000000),
                endPose,
                new ChassisSpeeds());
        setTrajectoryOnTheFly(trajectory, useAllianceColor);
    }

    public Pose2d sample(double timestamp){
        Pose2d pose = Pose2d.identity();
        if(mPathStateGenerator.sample(timestamp) != null)
            pose = mPathStateGenerator.sample(timestamp);
        return pose;
    }
    public Request generatePathRequest(Pose2d start, Pose2d end, ChassisSpeeds currentVelocity,
            boolean useAllianceColor) {
        return new Request() {
            @Override
            public void act() {
                generateAndPrepareTrajectory(start, end, currentVelocity, useAllianceColor);
            }
        };
    }

    public Request startPathRequest(boolean useAllianceColor) {

        return new Request() {
            @Override
            public void act() {
                startPath(useAllianceColor);
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

    public Request waitForTrajectoryRequest(double timestamp) {
        return new Request() {
            @Override
            public boolean isFinished() {
                if (mPathStateGenerator.getTime() == 0)
                    return false;
                return mPathStateGenerator.getTime() > timestamp;
            }
        };
    }
}