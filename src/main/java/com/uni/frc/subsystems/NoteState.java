package com.uni.frc.subsystems;


import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


import java.util.Map;
import java.util.Optional;

import javax.swing.text.html.Option;

import org.littletonrobotics.junction.Logger;

import com.uni.frc.Constants;
import com.uni.frc.Constants.VisionConstants;
import com.uni.frc.subsystems.Vision.VisionPoseAcceptor;
import com.uni.frc.subsystems.Vision.OdometryLimeLight.VisionUpdate;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Translation2d;
import com.uni.lib.geometry.Twist2d;
import com.uni.lib.util.InterpolatingDouble;
import com.uni.lib.util.InterpolatingTreeMap;
import com.uni.lib.util.MovingAverageTwist2d;
import com.uni.lib.util.Kalman.UnscentedKalmanFilter;

public class NoteState {
    private static NoteState mInstance;
    private Optional<VisionUpdate> mLatestVisionUpdate;
    private UnscentedKalmanFilter<N2, N2, N2> mKalmanFilter;
    private Pose2d mDisplayVisionPose;
    private RobotState mRobotState;

    public Field2d mField2d;

    public static NoteState getInstance() {
        if (mInstance == null) {
            mInstance = new NoteState();
        }

        return mInstance;
    }

    private static final int kObservationBufferSize = 50;
    
    private Optional<Translation2d> initialPose = Optional.empty();
    private InterpolatingTreeMap<InterpolatingDouble, Translation2d> visionPoseComponent;


    private NoteState() {
        mRobotState = RobotState.getInstance();
        reset(0.0, Pose2d.fromTranslation(new Translation2d(0,0)));
    }


    public synchronized void reset(double start_time, Pose2d initialPose) {
        visionPoseComponent = new InterpolatingTreeMap<>(kObservationBufferSize);
        visionPoseComponent.put(new InterpolatingDouble(start_time), initialPose.getTranslation());
        mLatestVisionUpdate = Optional.empty();
        mDisplayVisionPose = Pose2d.identity();
        this.initialPose = Optional.of(initialPose.getTranslation());
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }

    public synchronized void resetKalmanFilters(double timestamp) {
        visionPoseComponent = new InterpolatingTreeMap<>(kObservationBufferSize);
        visionPoseComponent.put(new InterpolatingDouble(timestamp), getInitialPose().getTranslation());
        mKalmanFilter =
        new UnscentedKalmanFilter<>(
            Nat.N2(),
            Nat.N2(),
            (x, u) -> VecBuilder.fill(0.0, 0.0),
            (x, u) -> x,
            Constants.kStateStdDevs,
            Constants.kLocalMeasurementStdDevs, .01);

    }

 
    public Pose2d getLatestNotePose(){
        return Pose2d.fromTranslation(visionPoseComponent.lastEntry().getValue());
    }

    public Pose2d getNotePose(double timestamp){
        return Pose2d.fromTranslation(visionPoseComponent.get(new InterpolatingDouble(timestamp)));
    }
    /**
     * Adds a Vision Update
     * @param visionUpdate
     */
    public synchronized void addVisionUpdate(VisionUpdate visionUpdate) {
        
        mLatestVisionUpdate = Optional.ofNullable(visionUpdate);

        if(!mLatestVisionUpdate.isEmpty()){
            
            double visionTimestamp = mLatestVisionUpdate.get().getTimestamp();

            Pose2d noteToCamera = Pose2d.fromTranslation(mLatestVisionUpdate.get().getCameraToTarget()).rotateBy(VisionConstants.cameraYawOffset);

            Pose2d robotToCamera =  mRobotState.getKalmanPose(visionTimestamp).transformBy(VisionConstants.ROBOT_TO_CAMERA);

            Pose2d noteToField = robotToCamera.transformBy(noteToCamera);

            mDisplayVisionPose = noteToField;

            try{
                mKalmanFilter.correct(VecBuilder.fill(0.0,0.0), VecBuilder.fill(noteToField.getTranslation().x(), noteToField.getTranslation().y()));
                visionPoseComponent.put(new InterpolatingDouble(visionTimestamp),new Translation2d(mKalmanFilter.getXhat(0), mKalmanFilter.getXhat(1)));
            } catch(Exception e){
                System.out.println(e.getStackTrace());
            }
        } else{
            mDisplayVisionPose = null;
        }
    }

    public synchronized Pose2d getDisplayVisionPose() {
        if (mDisplayVisionPose == null) {
            return Pose2d.fromTranslation(Translation2d.identity()); //Out of frame
        }
        return mDisplayVisionPose;
    }

    /**
     * Return Initial Vision Offset for Pure Odometry Visualization Purposes
     * @return
     */
    public synchronized Pose2d getInitialPose() {
        if (initialPose.isEmpty()) return Pose2d.identity();
        return Pose2d.fromTranslation(initialPose.get());

    }

    public synchronized Optional<VisionUpdate> getLatestVisionUpdate() {
        return mLatestVisionUpdate;
    }

    public void outputTelemetry() {
        Logger.recordOutput("Note Pose", getDisplayVisionPose().toWPI());
   }

}