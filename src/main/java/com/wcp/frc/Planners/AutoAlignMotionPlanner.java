package com.wcp.frc.Planners;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

import com.wcp.frc.Constants;
import com.wcp.lib.HeadingController;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.geometry.Twist2d;
import com.wcp.lib.motion.IMotionProfileGoal;
import com.wcp.lib.motion.MotionProfileGoal;
import com.wcp.lib.motion.MotionState;
import com.wcp.lib.motion.ProfileFollower;
import com.wcp.lib.swerve.ChassisSpeeds;

import edu.wpi.first.wpilibj.Timer;

public class AutoAlignMotionPlanner {
    
    private ProfileFollower mXController = new ProfileFollower(2.5, 0, 0, 1, 0, 0);
    private ProfileFollower mYController = new ProfileFollower(2.5, 0, 0, 1, 0, 0);
    private ProfileFollower mThetaController = new ProfileFollower(1.5, 0.0, 0.0, 1.0, 0.0, 0.0);

    private boolean mAutoAlignComplete = false;

    private Pose2d mFieldToTargetPoint;
    private OptionalDouble mStartTime;

    public void reset() {
        mStartTime = OptionalDouble.of(Timer.getFPGATimestamp());
        mXController.resetSetpoint();
        mYController.resetSetpoint();
        mXController.resetProfile();
        mYController.resetProfile();
        mAutoAlignComplete = false;
    }

    public synchronized void setTargetPoint(Pose2d targetPoint) {//TODO setpoint generator
        mFieldToTargetPoint = targetPoint;
    }

    public synchronized ChassisSpeeds updateAutoAlign(double timestamp, Pose2d currentOdomToVehicle,
            Pose2d currentFieldToOdom, Twist2d currentVel, HeadingController headingController, Rotation2d currentHeading) {
                var odomToTargetPoint = currentFieldToOdom.inverse().transformBy(mFieldToTargetPoint);
                Logger.recordOutput("offsetTargetPoint", odomToTargetPoint.toWPI());
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


        Translation2d currentVelocityFrame = new Translation2d(currentVel.dx, currentVel.dy);
        Translation2d currentVelocityOdometryFrame = currentVelocityFrame.rotateBy(currentOdomToVehicle.getRotation());


        double xOutput = -mXController.update(
                new MotionState(timestamp, currentOdomToVehicle.getTranslation().x(), currentVelocityOdometryFrame.x(),
                        0.0),
                timestamp + .01);
        double yOutput = -mYController.update(
                new MotionState(timestamp, currentOdomToVehicle.getTranslation().y(), currentVelocityOdometryFrame.y(),
                        0.0),
                timestamp + .01);
                headingController.setTargetHeading(mFieldToTargetPoint.getRotation());
        double thetaOutput = headingController.getRotationCorrection(currentHeading, timestamp);

        Translation2d output = new Translation2d(xOutput,yOutput).rotateBy(currentHeading);
        ChassisSpeeds setPoint;
                Logger.recordOutput("autoalignoutputs", output.toWPI());
        boolean yOutputWithinDeadband = mYController.onTarget();
        boolean xOutputWithinDeadband = mXController.onTarget();
        
        setPoint = ChassisSpeeds.fromFieldRelativeSpeeds(
                xOutputWithinDeadband ? 0.0 : output.x(),
                yOutputWithinDeadband ? 0.0 : output.y(),
                 thetaOutput,
                currentFieldToOdom.getRotation().rotateBy(currentHeading));
        mAutoAlignComplete = yOutputWithinDeadband && xOutputWithinDeadband;

        if (mStartTime.isPresent() && mAutoAlignComplete) {
            System.out.println("Alignment took " + (Timer.getFPGATimestamp() - mStartTime.getAsDouble()));
            mStartTime = OptionalDouble.empty();
        }

        return setPoint;
    }

    public synchronized boolean getAutoAlignmentCompleted() {
        return mAutoAlignComplete;
    }
}