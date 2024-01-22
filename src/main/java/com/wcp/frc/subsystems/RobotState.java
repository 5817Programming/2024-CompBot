// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.wcp.frc.Constants;
import com.wcp.frc.subsystems.Vision.VisionPoseAcceptor;
import com.wcp.frc.subsystems.Vision.LimeLight.VisionUpdate;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.geometry.Twist2d;
import com.wcp.lib.util.InterpolatingDouble;
import com.wcp.lib.util.InterpolatingTreeMap;
import com.wcp.lib.util.MovingAverageTwist2d;
import com.wcp.lib.util.Kalman.UnscentedKalmanFilter;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class RobotState {
    private static RobotState mInstance = null;
    private Optional<VisionUpdate> mLatestVisionUpdate;
    private UnscentedKalmanFilter<N2, N2, N2> mKalmanFilter;
    private VisionPoseAcceptor mPoseAcceptor;
    private Pose2d mDisplayVisionPose;

    private boolean mHasbeenEnabled = false;

    public static RobotState getInstance(){
        if(mInstance == null){
            return new RobotState();
        }

        return mInstance;
    }

    private static final int kObservationBufferSize = 50;

    private Optional<Translation2d> intitialFieldOdometry = Optional.empty();
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> odomToVehicle;
    private InterpolatingTreeMap<InterpolatingDouble, Translation2d> fieldToOdom;

    private Twist2d measuredVehicleVelocity;
    private Twist2d predictedVehicleVelocity;
    private MovingAverageTwist2d measuredVehicleVelocityFiltered;

    private RobotState(){
        reset(0.0, Pose2d.identity());
    }

    public synchronized void reset(double startTime, Pose2d initalPose){
        odomToVehicle = new InterpolatingTreeMap<>(kObservationBufferSize);
        odomToVehicle.put(new InterpolatingDouble(startTime), initalPose);
        fieldToOdom = new InterpolatingTreeMap<>(kObservationBufferSize);
        fieldToOdom.put(new InterpolatingDouble(startTime), getInitialFieldToOdom().getTranslation());
        predictedVehicleVelocity = Twist2d.identity();
        measuredVehicleVelocity = Twist2d.identity();
        measuredVehicleVelocityFiltered = new MovingAverageTwist2d(25);
        mLatestVisionUpdate = Optional.empty();
        mDisplayVisionPose = Pose2d.identity();
        mPoseAcceptor = new VisionPoseAcceptor(); 
    }

    public synchronized void reset(){
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }
    
    public synchronized void resetKalmanFilters(){
        mKalmanFilter = 
        new UnscentedKalmanFilter<>(
            Nat.N2(),
            Nat.N2(),
            (x, u) -> VecBuilder.fill(0.0,0.0),
            (x, u) -> x,
            Constants.kStateStdDevs,
            Constants.kLocalMeasurementStdDevs, 0.01);

    }
    
    public synchronized boolean getHasBeenEnabled() {
        return mHasbeenEnabled;
    }

    public synchronized void setHasBeenEnabled(boolean hasBeenEnabled) {
        mHasbeenEnabled = hasBeenEnabled;
    }
    
    public synchronized Pose2d getOdomToVehicle(double timestamp){
        return odomToVehicle.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestOdomToVehicle(){
        return odomToVehicle.lastEntry();
    }

    public synchronized Pose2d getPredictedOdomToVehicle(double lookahead_time){
        return getLatestOdomToVehicle().getValue()
                .transformBy(Pose2d.exp(predictedVehicleVelocity.scaled(lookahead_time)));
    }

    public synchronized void addOdomToVehicleObservation(double timestamp, Pose2d observation) {
        odomToVehicle.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addOdomObservation(double timestamp, Pose2d odometryToRobot, Twist2d measuredVelocity, Twist2d predictedVelocity){
        try { 
            mKalmanFilter.predict(VecBuilder.fill(0.0,0.0), 0.01);
        } catch (Exception e) {

        }
        addOdomToVehicleObservation(timestamp, odometryToRobot);

        measuredVehicleVelocity = measuredVelocity;
        measuredVehicleVelocityFiltered.add(measuredVehicleVelocity);
        predictedVehicleVelocity = predictedVelocity;
    }   

    public synchronized Twist2d getPredictedVelocity(){
        return predictedVehicleVelocity;
    }

    public synchronized Twist2d getMeasuredVelocity(){
        return measuredVehicleVelocity;
    }

    public synchronized Twist2d getSmoothedVelocity(){
        return measuredVehicleVelocityFiltered.getAverage();
    }

    public synchronized void addVisionUpdate(VisionUpdate visionUpdate){
        mLatestVisionUpdate = Optional.ofNullable(visionUpdate);
        if(!mLatestVisionUpdate.isEmpty()){

            double visionTimestamp = mLatestVisionUpdate.get().getTimestamp();

            Pose2d odomTovehicle = getOdomToVehicle(visionTimestamp);

            Pose2d cameraToTag = Pose2d.fromTranslation(mLatestVisionUpdate.get().getCameraToTarget().rotateBy(Constants.VisionConstants.cameraYawOffset));

            Pose2d vehicleToTag = Pose2d.fromTranslation(Constants.VisionConstants.ROBOT_TO_CAMERA.transformBy(cameraToTag).getTranslation().rotateBy(odomTovehicle.getRotation()));

            Pose2d visionFieldToVehicle = mLatestVisionUpdate.get().getFeildToTag().transformBy(vehicleToTag.inverse());

            if(!mPoseAcceptor.shouldAcceptVision(mLatestVisionUpdate.get().getTimestamp(), visionFieldToVehicle, vehicleToTag, measuredVehicleVelocity)){
                return;
            }

            boolean disabledAndNeverEnabled = DriverStation.isDisabled() && !mHasbeenEnabled;
            if(intitialFieldOdometry.isEmpty() || disabledAndNeverEnabled){
                var odometryToVehicleTranslation = disabledAndNeverEnabled ? Translation2d.identity() : getOdomToVehicle(visionTimestamp).getTranslation();
                fieldToOdom.put(new InterpolatingDouble(visionTimestamp), visionFieldToVehicle.getTranslation().translateBy(odometryToVehicleTranslation.inverse()));
                intitialFieldOdometry = Optional.of(fieldToOdom.lastEntry().getValue());
                mKalmanFilter.setXhat(0, fieldToOdom.lastEntry().getValue().x());
                mKalmanFilter.setXhat(1, fieldToOdom.lastEntry().getValue().y());
                mDisplayVisionPose = visionFieldToVehicle;

            } else if(DriverStation.isEnabled()){
                var fieldToOdomTranslation = visionFieldToVehicle.getTranslation().translateBy(odomTovehicle.getTranslation().inverse());
                if(DriverStation.isAutonomous()) {
                    final double kMaxDistanceToAccept = 2.0;
                    if(fieldToOdomTranslation.inverse().translateBy(fieldToOdom.lastEntry().getValue()).norm() > kMaxDistanceToAccept){
                        System.out.print("invalid vision update");
                        return;
                    }
                }
                
                mDisplayVisionPose = visionFieldToVehicle;
                try{
                    mKalmanFilter.correct(VecBuilder.fill(0.0,0.0), VecBuilder.fill(fieldToOdomTranslation.getTranslation().x(), fieldToOdomTranslation.getTranslation().y()));
                    fieldToOdom.put(new InterpolatingDouble(visionTimestamp), Pose2d.fromTranslation(new Translation2d(mKalmanFilter.getXhat(0), mKalmanFilter.getXhat(1))).getTranslation());
                } catch (Exception e){
                    DriverStation.reportError("Qr Decomposition failed", e.getStackTrace());
                }
            }else{
                mDisplayVisionPose = null;
            }
        }
    }

    public synchronized Pose2d getDisplayVisionPose(){
        if(mDisplayVisionPose == null){
            return Pose2d.identity();
        }
        return mDisplayVisionPose;
    }

    public synchronized Pose2d getInitialFieldToOdom() {
        if (intitialFieldOdometry.isEmpty()) return Pose2d.identity();
        return Pose2d.fromTranslation(intitialFieldOdometry.get());
    }

    public synchronized Translation2d getFieldToOdom(double timestamp){
        if (intitialFieldOdometry.isEmpty()) return Translation2d.identity();
        return intitialFieldOdometry.get().inverse().translateBy(fieldToOdom.getInterpolated(new InterpolatingDouble(timestamp)));
    }

    public synchronized Translation2d getAbsoluteFieldToOdom(double timestamp){
        return fieldToOdom.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Translation2d getLatestFieldToOdom(){
        return getFieldToOdom(fieldToOdom.lastKey().value);
    }

    public synchronized Pose2d getFieldToVehicle(double timestamp){
        Pose2d odomToVehicle = getOdomToVehicle(timestamp);
        Translation2d fieldToOdom = getFieldToOdom(timestamp);
        return new Pose2d(fieldToOdom.translateBy(odomToVehicle.getTranslation()), odomToVehicle.getRotation());
    }

    public synchronized Pose2d getFieldToVehicleAbsolute(double timestamp){
        var fieldToOdom = intitialFieldOdometry.orElse(Translation2d.identity());
        return Pose2d.fromTranslation(fieldToOdom).transformBy(getFieldToVehicle(timestamp));
    }

    public synchronized Pose2d getLatestFieldToVehicle(){
        Pose2d odomToVehicle = getLatestOdomToVehicle().getValue();
        return new Pose2d(getLatestFieldToOdom().getTranslation().add(odomToVehicle.getTranslation()), odomToVehicle.getRotation());
    }

    public synchronized Optional<VisionUpdate> getLatestVisionUpdate(){
        return mLatestVisionUpdate;
    }

    public void outputTelemetry(){
        Logger.recordOutput("Robot Velocity", getMeasuredVelocity().toString());
        Logger.recordOutput("Odom to Robot",  getOdomToVehicle(Timer.getFPGATimestamp()).toWPI());
        Logger.recordOutput("Field To Robot", getFieldToOdom(Timer.getFPGATimestamp()).toWPI());

    }
}
