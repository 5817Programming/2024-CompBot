// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.Planners;

public class DriveMotionPlanner{

    private Translation2d targetFollowTranslation = new Translation2d();
    private Rotation2d targetHeading = new Rotation2d();

    PathFollower pathFollower;
    
    public DriveMotionPlanner(){
        pathFollower = PathFollower.getInstance();

    }

    public void setTrajectory(PathPlannerTrajectory trajectory, double nodes,double initRotation) {
        trajectoryFinished = false;
        pathFollower.setTrajectory(trajectory, nodes);
        poseMeters = pathFollower.getInitial(trajectory,initRotation);
        Pose2d newpose = (pathFollower.getInitial(trajectory,initRotation));
        modules.forEach((m) -> m.resetPose(new Pose2d(newpose.getTranslation(), new Rotation2d())));
        gyro.setAngle(newpose.getRotation().getDegrees());

    }

    public void startPath(boolean useAllianceColor) {
        trajectoryStarted = true;
        this.useAllianceColor = useAllianceColor;
        pathFollower.startTimer();

    }

    public void resetTimer() {
        PathFollower.getInstance().resetTimer();
    }

      public void updateTrajectory() {
        Pose2d desiredPose = pathFollower.getDesiredPose2d(useAllianceColor, getPoseMeters());
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
            setState(State.OFF);
            trajectoryFinished = true;
            return new Translation2d();
        }
        return new Translation2d(xError, -yError);
    }


        public Request waitForTrajectoryRequest(double PercentageToRun) {
        return new Request() {
            @Override
            public boolean isFinished() {
                return pathFollower.hasElapsedPercentage(PercentageToRun);
            }
        };
    }
}
 