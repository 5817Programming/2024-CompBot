// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.Planners;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.subsystems.RobotState;
import com.uni.frc.subsystems.RobotStateEstimator;
import com.uni.frc.subsystems.Requests.Request;
import com.uni.frc.subsystems.Vision.ObjectLimeLight.VisionObjectUpdate;
import com.uni.frc.subsystems.gyros.Pigeon;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Translation2d;
import com.uni.lib.motion.PathGenerator;
import com.uni.lib.motion.PathStateGenerator;
import com.uni.lib.swerve.ChassisSpeeds;
import com.uni.lib.util.PID2d;
import com.uni.lib.util.SynchronousPIDF;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DriveMotionPlanner {

    boolean useAllianceColor;
    private Translation2d targetFollowTranslation = new Translation2d();
    private Rotation2d targetHeading = new Rotation2d();

    boolean trajectoryStarted = false;
    boolean trajectoryFinished = false;
    Pose2d drivingpose = new Pose2d();
    PathPlannerTrajectory trajectoryDesired;
    PathStateGenerator mPathStateGenerator;
    PathGenerator mPathGenerator;
    PID2d OdometryPID = new PID2d(new SynchronousPIDF(.3, 0, 0), new SynchronousPIDF(.3, 0, 0));
    SynchronousPIDF mTrackingPID = new SynchronousPIDF(.005 ,0 ,0);
    double lastTimestamp = 0;
    public static DriveMotionPlanner instance = null;

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
        // Pose2d newpose = (mPathStateGenerator.getInitial(trajectory, initRotation, useAllianceColor));
        // RobotStateEstimator.getInstance().resetOdometry(newpose);
        Pigeon.getInstance().setAngle(Rotation2d.fromDegrees(initRotation).getDegrees());
        if(DriverStation.getAlliance().get().equals(Alliance.Red))
        Pigeon.getInstance().setAngle(Rotation2d.fromDegrees(initRotation).flip().inverse().getDegrees());

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
        Pose2d desiredPose = mPathStateGenerator.getDesiredPose2d(useAllianceColor);
        targetHeading = desiredPose.getRotation().inverse();
        targetFollowTranslation = desiredPose.getTranslation();
    }

    public Translation2d getTranslation2dToFollow(double timestamp) {
        double dt = timestamp - lastTimestamp;
        Translation2d currentRobotPositionFromStart = RobotState.getInstance().getLatestKalmanPose()
                .getTranslation();
        OdometryPID.x().setOutputRange(-1, 1);
        OdometryPID.y().setOutputRange(-1, 1);
        Logger.recordOutput("Desired Pose", Pose2d.fromTranslation(targetFollowTranslation).toWPI());
        double xError = OdometryPID.x().calculate(targetFollowTranslation.x() - currentRobotPositionFromStart.x(), dt);
        double yError = OdometryPID.y().calculate(targetFollowTranslation.y() - currentRobotPositionFromStart.y(), dt);
        lastTimestamp = timestamp;
        if (((Math.abs(xError) + Math.abs(yError)) / 2 < .05 && PathStateGenerator.getInstance().isFinished())
                || trajectoryFinished) {
            trajectoryFinished = true;
            return new Translation2d();
        }
        return new Translation2d(xError, -yError);
    }
    public Translation2d getTranslation2dToTrack(double timestamp, Optional<VisionObjectUpdate> visionUpdate) {
        double dt = timestamp - lastTimestamp;
        Pose2d currentPose = RobotState.getInstance().getLatestKalmanPose();
        Translation2d visionCompensation = Translation2d.identity();
        if(!visionUpdate.isEmpty()){
            double yComp = mTrackingPID.calculate(-visionUpdate.get().getCameraToTarget().x(), dt);
            visionCompensation = new Translation2d(0,-yComp).rotateBy(currentPose.getRotation().inverse());
        }
        Translation2d targetTranslation = targetFollowTranslation.translateBy(visionCompensation);
        Logger.recordOutput("desiredPose", Pose2d.fromTranslation(targetTranslation).toWPI());
        OdometryPID.x().setOutputRange(-1, 1);
        OdometryPID.y().setOutputRange(-1, 1);
        double xError = OdometryPID.x().calculate(targetTranslation.x() - currentPose.getTranslation().x(), dt);
        double yError = OdometryPID.y().calculate(targetTranslation.y() - currentPose.getTranslation().y(), dt);
        lastTimestamp = timestamp;
        if (((Math.abs(xError) + Math.abs(yError)) / 2 < .05 && PathStateGenerator.getInstance().isFinished())
                || trajectoryFinished) {
            trajectoryFinished = true;
            return new Translation2d();
        }
        return new Translation2d(xError, -yError);
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
                System.out.println("running");
                if (mPathStateGenerator.getTime() == 0)
                    return false;
                return mPathStateGenerator.getTime() > timestamp;
            }
        };
    }

  

}
