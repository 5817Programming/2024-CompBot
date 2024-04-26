package com.uni.frc.Planners;

import com.uni.lib.HeadingController;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Translation2d;
import com.uni.lib.motion.PathStateGenerator;
import com.uni.lib.util.PID2d;
import com.uni.lib.util.SynchronousPIDF;

public class DriveMotionPlanner{
    private static DriveMotionPlanner instance = null;
    private PathStateGenerator mPathStateGenerator;
    private PID2d translationPID;
    private double lastTimeStamp = Double.MIN_VALUE;

    private boolean trajectoryFinished;
    private boolean noteTrackFinished;

    public static DriveMotionPlanner getInstance(){
        if(instance == null)
            instance = new DriveMotionPlanner();
        return instance;
    }

    public DriveMotionPlanner(){
        mPathStateGenerator = PathStateGenerator.getInstance();
        translationPID = new PID2d(
                        new SynchronousPIDF(.3,0,0,.9), 
                        new SynchronousPIDF(.3,0,0,.9));

        trajectoryFinished = false;
        noteTrackFinished = false;
    }

    public Pose2d getPoseControl(HeadingController headingController, Pose2d currentPose, double timestamp){
        double dt = timestamp = lastTimeStamp;

        if(mPathStateGenerator.getDesiredPose2d().isEmpty())
            return new Pose2d(0,0, currentPose.getRotation().inverse());

        Pose2d desiredPose = mPathStateGenerator.getDesiredPose2d().get();
        
        double dx = desiredPose.getTranslation().x() - desiredPose.getTranslation().x();
        double x = translationPID.x().calculate(dx, dt);


        double dy = desiredPose.getTranslation().y() - desiredPose.getTranslation().y();
        double y = translationPID.y().calculate(dy, dt);

        double dtheta = desiredPose.getRotation().getDegrees() - currentPose.getRotation().inverse().getDegrees();
        headingController.setTargetHeading(desiredPose.getRotation());
        double theta = headingController.getRotationCorrection(currentPose.getRotation(), timestamp);

        boolean xFinished = Math.abs(dx) <.1;
        boolean yFinished = Math.abs(dy) <.1;
        boolean thetaFinished = Math.abs(dtheta) <1;
        
        trajectoryFinished = xFinished && yFinished && thetaFinished;

        lastTimeStamp = timestamp;

        return new Pose2d(
            new Translation2d(
                xFinished ? 0.0 : x,
                yFinished ? 0.0 : y
            ),
            Rotation2d.fromDegrees(
                thetaFinished ? 0.0 : theta
            )
        );
    }

    public Pose2d getNoteTrackingControl(HeadingController headingController, Pose2d currentPose, Pose2d notePose, double timestamp){
        if(notePose.getTranslation().translateBy(currentPose.getTranslation()).norm() < 1)
            return getPoseControl(headingController, currentPose, timestamp);

        double dt = timestamp - lastTimeStamp;
        
        
        
        double dx = notePose.getTranslation().x() - notePose.getTranslation().x();
        double x = translationPID.x().calculate(dx, dt);


        double dy = notePose.getTranslation().y() - notePose.getTranslation().y();
        double y = translationPID.y().calculate(dy, dt);

        double thetaSetpoint = notePose.getTranslation().translateBy(currentPose.getTranslation().inverse()).getAngle().inverse().flip().getDegrees();
        double dtheta = thetaSetpoint - currentPose.getRotation().getDegrees();
        headingController.setTargetHeading(notePose.getRotation());
        double theta = headingController.getRotationCorrection(currentPose.getRotation(), timestamp);

        boolean xFinished = Math.abs(dx) <.1;
        boolean yFinished = Math.abs(dy) <.1;
        boolean thetaFinished = Math.abs(dtheta) <1;
        
        noteTrackFinished = xFinished && yFinished && thetaFinished;

        lastTimeStamp = timestamp;

        return new Pose2d(
            new Translation2d(
                xFinished ? 0.0 : x,
                yFinished ? 0.0 : y
            ),
            Rotation2d.fromDegrees(
                thetaFinished ? 0.0 : theta
            )
        );
        
    }

    public boolean trajectoryFinished(){
        return trajectoryFinished;
    }

    public boolean noteTrackFinished(){
        return noteTrackFinished;
    }

    public void resetTrajectory(){
        trajectoryFinished = false;
        mPathStateGenerator.resetTrajectory();
    }

}