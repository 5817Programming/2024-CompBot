package com.wcp.frc.Planners;

import java.nio.file.Path;
import java.util.OptionalDouble;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.wcp.frc.Constants;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.geometry.Twist2d;
import com.wcp.lib.motion.IMotionProfileGoal;
import com.wcp.lib.motion.MotionProfileGoal;
import com.wcp.lib.motion.MotionState;
import com.wcp.lib.motion.ProfileFollower;
import com.wcp.lib.swerve.ChassisSpeeds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class DriveMotionPlanner {
    
    private ProfileFollower mXController = new ProfileFollower(2.5, 0, 0, 1, 0, 0);
    private ProfileFollower mYController = new ProfileFollower(2.5, 0, 0, 1, 0, 0);
    private ProfileFollower mThetaController = new ProfileFollower(1.5, 0.0, 0.0, 1.0, 0.0, 0.0);

    private boolean mTrajectoryFollowed = false;

    private Pose2d mTargetPose;
    private PathPlannerTrajectory mTrajectory;
    private OptionalDouble mStartTime;
    private Rotation2d mTargetHeading;
    private final double FIELD_WIDTH_METERS = 8.02;

    public void startTrajectory() {
        mStartTime = OptionalDouble.of(Timer.getFPGATimestamp());
        mXController.resetSetpoint();
        mYController.resetSetpoint();
        mXController.resetProfile();
        mYController.resetProfile();
        mTrajectoryFollowed = false;
    }

    public synchronized void setTargetTrajectory(PathPlannerTrajectory trajectory) {//TODO setpoint generator
        mTrajectory = trajectory;
    }

    public synchronized Pose2d transformPoseForAlliance(State state, DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Red) {
        Translation2d transformedTranslation =
                new Translation2d(state.positionMeters.getX(), FIELD_WIDTH_METERS - state.positionMeters.getY());
        Rotation2d transformedHolonomicRotation = Rotation2d.fromDegrees(state.targetHolonomicRotation.times(-1).getDegrees());

        return new Pose2d(transformedTranslation, transformedHolonomicRotation);
        } else {
        Translation2d transformedTranslation =
                new Translation2d(state.positionMeters.getX(), state.positionMeters.getY());
        Rotation2d transformedHolonomicRotation = Rotation2d.fromDegrees(state.targetHolonomicRotation.getDegrees());
        return new Pose2d(transformedTranslation, transformedHolonomicRotation);
        }
    }

    public synchronized Rotation2d transformHeadingForAlliance(State state, DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Red) {
        return Rotation2d.fromDegrees(state.heading.times(-1).getDegrees());
        } else {
        return Rotation2d.fromDegrees(state.heading.getDegrees());
        }
    }

    public synchronized DriverStation.Alliance getAlliance(){
        return DriverStation.getAlliance().get();
    }

    


    public synchronized ChassisSpeeds updateAutoAlign(double timestamp, Pose2d currentOdomToVehicle,
        Pose2d currentFieldToOdom, Twist2d currentVel) {
                
        double sampleTime;
        if(mStartTime.isEmpty())
        sampleTime = 0;
        else{
        sampleTime = timestamp - mStartTime.getAsDouble();
        }

        var targetState = (PathPlannerTrajectory.State) mTrajectory.sample(sampleTime);
        var targetVelocity = targetState.velocityMps;
        mTargetPose = transformPoseForAlliance(targetState, getAlliance());
        mTargetHeading = transformHeadingForAlliance(targetState, getAlliance());

        double xFF =
                targetVelocity* mTargetHeading.getRotation().cos();
        double yFF =
                targetVelocity * mTargetHeading.getRotation().sin();
        double rotationFF = 
                targetState.holonomicAngularVelocityRps.get();

        var odomToTargetPoint = currentFieldToOdom.inverse().transformBy(mTargetPose);

        mXController.setGoalAndConstraints(
                new MotionProfileGoal(odomToTargetPoint.getTranslation().x(), 0,
                        IMotionProfileGoal.CompletionBehavior.VIOLATE_MAX_ACCEL, 0.08, 0.05),
                Constants.kPositionMotionProfileConstraints);
        mYController.setGoalAndConstraints(
                new MotionProfileGoal(odomToTargetPoint.getTranslation().y(), 0,
                        IMotionProfileGoal.CompletionBehavior.OVERSHOOT, 0.08, 0.05),
                Constants.kPositionMotionProfileConstraints);
        mThetaController.setGoalAndConstraints(
                new MotionProfileGoal(odomToTargetPoint.getRotation().getRadians(), 0,
                        IMotionProfileGoal.CompletionBehavior.OVERSHOOT, 0.03, 0.05),
                Constants.kHeadingMotionProfileConstraints);
        double currentRotation = currentFieldToOdom.getRotation().rotateBy(currentOdomToVehicle.getRotation())
                .getRadians();

        Translation2d currentVelocityFrame = new Translation2d(currentVel.dx, currentVel.dy);
        Translation2d currentVelocityOdometryFrame = currentVelocityFrame.rotateBy(currentOdomToVehicle.getRotation());

        if (odomToTargetPoint.getRotation().getRadians() - currentRotation > Math.PI) {
            currentRotation += 2 * Math.PI;
        } else if (odomToTargetPoint.getRotation().getRadians() - currentRotation < -Math.PI) {
            currentRotation -= 2 * Math.PI;
        }

        double xOutput = mXController.update(
                new MotionState(timestamp, currentOdomToVehicle.getTranslation().x(), currentVelocityOdometryFrame.x(),
                        0.0),
                timestamp + .01);
        double yOutput = mYController.update(
                new MotionState(timestamp, currentOdomToVehicle.getTranslation().y(), currentVelocityOdometryFrame.y(),
                        0.0),
                timestamp + .01);
        double thetaOutput = mThetaController
                .update(new MotionState(timestamp, currentRotation, currentVel.dtheta, 0.0), timestamp + .01);

        ChassisSpeeds setPoint;

        boolean thetaWithinDeadband = mThetaController.onTarget();
        boolean yOutputWithinDeadband = mYController.onTarget();
        boolean xOutputWithinDeadband = mXController.onTarget();
        boolean mTrajectoryFollowed = thetaWithinDeadband && yOutputWithinDeadband && xOutputWithinDeadband && timestamp >= mTrajectory.getTotalTimeSeconds();
        setPoint = ChassisSpeeds.fromFieldRelativeSpeeds(
                (xOutputWithinDeadband ? 0.0 : xOutput) + xFF,
                (yOutputWithinDeadband ? 0.0 : yOutput) + yFF,
                (thetaWithinDeadband ? 0.0 : thetaOutput) + rotationFF,
                currentFieldToOdom.getRotation().rotateBy(currentOdomToVehicle.getRotation()));
        mTrajectoryFollowed = yOutputWithinDeadband && xOutputWithinDeadband && thetaWithinDeadband;

        if (mStartTime.isPresent() && mTrajectoryFollowed) {
            System.out.println("Alignment took " + (Timer.getFPGATimestamp() - mStartTime.getAsDouble()));
            mStartTime = OptionalDouble.empty();
        }

        return setPoint;
    }

    public boolean pastPercent(double timestamp, double percent){
        double currentPercent = timestamp - mStartTime.getAsDouble();
        return currentPercent >= percent;
    }

    public synchronized boolean getAutoAlignmentCompleted() {
        return mTrajectoryFollowed;
    }
}