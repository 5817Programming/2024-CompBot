// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.Planners;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.wcp.frc.subsystems.RobotState;
import com.wcp.frc.subsystems.Requests.Request;
import com.wcp.frc.subsystems.Swerve.SwerveDrive;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.motion.PathFollower;
import com.wcp.lib.util.PID2d;
import com.wcp.lib.util.SynchronousPIDF;

import edu.wpi.first.wpilibj.Timer;


public class DriveMotionPlanner{

    boolean useAllianceColor;
    private Translation2d targetFollowTranslation = new Translation2d();
    private Rotation2d targetHeading = new Rotation2d();

    boolean trajectoryStarted = false;
    boolean trajectoryFinished = false;
    Pose2d drivingpose = new Pose2d();
    PathPlannerTrajectory trajectoryDesired;
    PathFollower pathFollower;
    PID2d OdometryPID = new PID2d(new SynchronousPIDF(0.003,0,0),new SynchronousPIDF(0.003,0,0));//TODO
    SwerveDrive swerve;
    double lastTimestamp = 0;
    Pose2d poseMeters;
    
    public DriveMotionPlanner(){
        pathFollower = PathFollower.getInstance();
        swerve = SwerveDrive.getInstance();
    }
    public Rotation2d getTargetHeading(){
        return targetHeading;
    }

    public void setTrajectory(PathPlannerTrajectory trajectory, double nodes,double initRotation) {
        trajectoryFinished = false;
        pathFollower.setTrajectory(trajectory, nodes);
        poseMeters = pathFollower.getInitial(trajectory,initRotation);
        Pose2d newpose = (pathFollower.getInitial(trajectory,initRotation));
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), newpose);
    }

    public void startPath(boolean useAllianceColor) {
        trajectoryStarted = true;
        this.useAllianceColor = useAllianceColor;
        pathFollower.startTimer();

    }

    public void resetTimer() {
        PathFollower.getInstance().resetTimer();
    }
    public Request setTrajectoryRequest(PathPlannerTrajectory trajectory, double nodes,double initRotation) {
        return new Request() {

            @Override
            public void act() {
                setTrajectory(trajectory, nodes,initRotation);
            }

        };
    }

      public void updateTrajectory() {
        Pose2d desiredPose = pathFollower.getDesiredPose2d(useAllianceColor, swerve.getPoseMeters());
        targetHeading = desiredPose.getRotation().inverse();
        targetFollowTranslation = desiredPose.getTranslation();
    }


        public Translation2d updateFollowedTranslation2d(double timestamp) {
        double dt = timestamp - lastTimestamp;
        Translation2d currentRobotPositionFromStart = poseMeters.getTranslation();
        OdometryPID.x().setOutputRange(-.9, .9);
        OdometryPID.y().setOutputRange(-.9, .9);
        double xError = OdometryPID.x().calculate(targetFollowTranslation.x() - currentRobotPositionFromStart.x(),
                dt);
        double yError = OdometryPID.y().calculate(targetFollowTranslation.y() - currentRobotPositionFromStart.y(),
                dt);
        lastTimestamp = timestamp;
        if (Math.abs(xError + yError) / 2 < .1 && PathFollower.getInstance().isFinished()) {
            trajectoryFinished = true;
            return new Translation2d();
        }
        return new Translation2d(xError, -yError);
    }
    // public Request generateTrajectoryRequest(int node) {
    // return new Request() {

    // @Override
    // public void act() {
    // PathPlannerTrajectory trajectory = PathGenerator.generatePath(new
    // PathConstraints(4, 4),
    // new Node(Constants.scoresY.get(node), DriverStation.getAlliance() ==
    // Alliance.Blue ? 2 : 14.71),
    // Constants.FieldConstants.obstacles);
    // setTrajectory(trajectory);TODO
    // }

    // };

    // }

    // public Request generateTrajectoryRequest(Node node) {
    // return new Request() {

    // @Override
    // public void act() {
    // PathPlannerTrajectory trajectory = PathGenerator.generatePath(new
    // PathConstraints(4, 4), node,
    // Constants.FieldConstants.obstacles);
    // setTrajectory(trajectory);
    // }

    // };

    // }

public Request startPathRequest(boolean useAllianceColor) {

        return new Request() {
            @Override
            public void act() {
                startPath(useAllianceColor);
            }
        };
    }

        public Request waitForTrajectoryRequest(double PercentageToRun) {
        return new Request() {
            @Override
            public boolean isFinished() {
                return pathFollower.hasElapsedPercentage(PercentageToRun);
            }
        };}
    public Request waitForTrajectoryRequest() {
        return new Request() {
            @Override
            public boolean isFinished() {
                return trajectoryFinished;
            }
        };
    }
    
}
 