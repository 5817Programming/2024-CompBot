package com.uni.frc.subsystems;


import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.uni.frc.Constants;
import com.uni.frc.subsystems.Vision.VisionPoseAcceptor;
import com.uni.frc.subsystems.Vision.OdometryLimeLight.VisionUpdate;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Translation2d;
import com.uni.lib.geometry.Twist2d;
import com.uni.lib.util.InterpolatingDouble;
import com.uni.lib.util.InterpolatingTreeMap;
import com.uni.lib.util.MovingAverageTwist2d;
import com.uni.lib.util.Kalman.UnscentedKalmanFilter;

public class RobotState {
    private static RobotState mInstance;
    private Optional<VisionUpdate> mLatestVisionUpdate;
    private UnscentedKalmanFilter<N2, N2, N2> mKalmanFilter;
    private VisionPoseAcceptor mPoseAcceptor;
    private Pose2d mDisplayVisionPose;
    private Pose2d mSetpointPose;

    public Field2d mField2d;

    private boolean mHasBeenEnabled = false;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }

        return mInstance;
    }

    private static final int kObservationBufferSize = 50;
    
    private Optional<Translation2d> initialPose = Optional.empty();
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> poseFromOdom;
    private InterpolatingTreeMap<InterpolatingDouble, Translation2d> visionPoseComponent;

    private Twist2d PredictedVelocity;
    private Twist2d MeasuredVelocity;
    private MovingAverageTwist2d filteredMeasuredVelocity;

    private RobotState() {
        reset(0.0, Pose2d.fromTranslation(new Translation2d(0,0)));
    }


    public synchronized void reset(double start_time, Pose2d initialPose) {
        poseFromOdom = new InterpolatingTreeMap<>(kObservationBufferSize);
        poseFromOdom.put(new InterpolatingDouble(start_time), initialPose);
        visionPoseComponent = new InterpolatingTreeMap<>(kObservationBufferSize);
        visionPoseComponent.put(new InterpolatingDouble(start_time), getInitialPose().getTranslation());
        PredictedVelocity = Twist2d.identity();
        MeasuredVelocity = Twist2d.identity();
        filteredMeasuredVelocity = new MovingAverageTwist2d(25);
        mLatestVisionUpdate = Optional.empty();
        mDisplayVisionPose = Pose2d.identity();
        mSetpointPose = Pose2d.identity();
        mPoseAcceptor = new VisionPoseAcceptor();
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }

    public synchronized void resetKalmanFilters() {
        mKalmanFilter =
        new UnscentedKalmanFilter<>(
            Nat.N2(),
            Nat.N2(),
            (x, u) -> VecBuilder.fill(0.0, 0.0),
            (x, u) -> x,
            Constants.kStateStdDevs,
            Constants.kLocalMeasurementStdDevs, .01);

    }

    public synchronized boolean getHasBeenEnabled() {
        return mHasBeenEnabled;
    }

    public synchronized void setHasBeenEnabled(boolean hasBeenEnabled) {
        mHasBeenEnabled = hasBeenEnabled;
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2d getPoseFromOdom(double timestamp) {
        return poseFromOdom.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestPoseFromOdom() {
        return poseFromOdom.lastEntry();
    }

    public synchronized Pose2d getPredictedPoseFromOdometry(double lookahead_time) {
        return getLatestPoseFromOdom().getValue()
                .transformBy(Pose2d.projectTwist(PredictedVelocity.scaled(lookahead_time)));
    }

    public synchronized void addPoseObservation(double timestamp, Pose2d observation) {
        poseFromOdom.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addOdomObservations(double timestamp, Pose2d poseFromOdom, Twist2d measured_velocity, Twist2d predicted_velocity) {
            mKalmanFilter.predict(VecBuilder.fill(0.0, 0.0), .01);

              
        addPoseObservation(timestamp, poseFromOdom);
        MeasuredVelocity = measured_velocity;
        filteredMeasuredVelocity.add(MeasuredVelocity);
        PredictedVelocity = new Twist2d(predicted_velocity.dx, -predicted_velocity.dy, predicted_velocity.dtheta);
    }

    public synchronized Twist2d getPredictedVelocity() {
        return PredictedVelocity;
    }

    public synchronized Twist2d getMeasuredVelocity() {
        return MeasuredVelocity;
    }

    public synchronized Twist2d getSmoothedVelocity() {
        return filteredMeasuredVelocity.getAverage();
    }

    /**
     * Adds a Vision Update
     * @param visionUpdate
     */
    public synchronized void addVisionUpdate(VisionUpdate visionUpdate) {
        mLatestVisionUpdate = Optional.ofNullable(visionUpdate);
        if (!mLatestVisionUpdate.isEmpty()) {
            //Get the Timestamp of the Vision Reading
            double visionTimestamp = mLatestVisionUpdate.get().getTimestamp();
            
            Pose2d odomToVehicle = getPoseFromOdom(visionTimestamp);
            //Rotating Camera by Yaw Offset
            Pose2d cameraToTag = Pose2d.fromTranslation(mLatestVisionUpdate.get().getCameraToTarget().rotateBy(Constants.VisionConstants.cameraYawOffset.flip()));
            //Getting Vehicle to Tag in Field Frame
            Pose2d vehicleToTag = Pose2d.fromTranslation(Constants.VisionConstants.ROBOT_TO_CAMERA.transformBy(cameraToTag).getTranslation().rotateBy(odomToVehicle.getRotation().inverse()));

            //Getting Field to Vehicle via Vehicle to Tag
            Pose2d visionFieldToVehicle = mLatestVisionUpdate.get().getFieldToTag().transformBy(vehicleToTag.inverse());

            if (!mPoseAcceptor.shouldAcceptVision(mLatestVisionUpdate.get().getTimestamp(), visionFieldToVehicle, vehicleToTag, MeasuredVelocity)) {
                return;
            }

            boolean disabledAndNeverEnabled = DriverStation.isDisabled() && !mHasBeenEnabled;
            if (initialPose.isEmpty() || disabledAndNeverEnabled) {
                var odom_to_vehicle_translation = disabledAndNeverEnabled ? Translation2d.identity() : getPoseFromOdom(visionTimestamp).getTranslation();
                visionPoseComponent.put(new InterpolatingDouble(visionTimestamp), visionFieldToVehicle.getTranslation().translateBy(odom_to_vehicle_translation.inverse()));
                initialPose = Optional.of(visionPoseComponent.lastEntry().getValue());
                mKalmanFilter.setXhat(0, visionPoseComponent.lastEntry().getValue().x());
                mKalmanFilter.setXhat(1, visionPoseComponent.lastEntry().getValue().y());
                mDisplayVisionPose = visionFieldToVehicle;

            } else if (DriverStation.isEnabled()) { 
                var visionOdomError = visionFieldToVehicle.getTranslation().translateBy(odomToVehicle.getTranslation().inverse());
                Logger.recordOutput("visionOdomError", visionOdomError.toWPI());

                if(DriverStation.isAutonomous()) {
                    final double kMaxDistanceToAccept = .5;
                    if (visionOdomError.norm() > kMaxDistanceToAccept) {
                        System.out.println("Invalid vision update!");
                        return;
                    }
                }
                mDisplayVisionPose = visionFieldToVehicle;
                try {
                    mKalmanFilter.correct(VecBuilder.fill(0.0, 0.0), VecBuilder.fill(visionOdomError.getTranslation().x(), visionOdomError.getTranslation().y()));
                    visionPoseComponent.put(new InterpolatingDouble(visionTimestamp), Pose2d.fromTranslation(new Translation2d(mKalmanFilter.getXhat(0), mKalmanFilter.getXhat(1))).getTranslation());
                } catch (Exception e) {
                    DriverStation.reportError("QR Decomposition failed: ", e.getStackTrace());
                }
            } else {
                mDisplayVisionPose = null;
            }
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

    public synchronized Translation2d getVisionPoseComponent(double timestamp) {
        if (initialPose.isEmpty()) return Translation2d.identity();
        return initialPose.get().inverse().translateBy(visionPoseComponent.getInterpolated(new InterpolatingDouble(timestamp)));
    }

    public synchronized Translation2d getAbsoluteVisionPoseComponent(double timestamp) {
        return visionPoseComponent.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Translation2d getLatestVisionPoseComponent() {
        return getAbsoluteVisionPoseComponent(visionPoseComponent.lastKey().value);
    }

    /**
     * Get Current Field to Vehicle using Filter Idea 1 (Offset Space) => Add the Offset outputted by the Filter to Current Odom
     * @param timestamp
     * @return
     */
    public synchronized Pose2d getKalmanPose(double timestamp) {
        Pose2d poseFromOdom = getPoseFromOdom(timestamp);

        Translation2d kalmanPoseOffset = getAbsoluteVisionPoseComponent(timestamp);
        return new Pose2d(kalmanPoseOffset.translateBy(poseFromOdom.getTranslation()), poseFromOdom.getRotation());

    }

    public synchronized Pose2d getAbosoluteKalmanPose(double timestamp) {
        var initialPose_ = initialPose.orElse(Translation2d.identity());
        return Pose2d.fromTranslation(initialPose_).transformBy(getKalmanPose(timestamp));
    }

    public synchronized Pose2d getLatestKalmanPose() {
        Pose2d poseFromOdom = getLatestPoseFromOdom().getValue();
        return new Pose2d(getLatestVisionPoseComponent().getTranslation().add(poseFromOdom.getTranslation()), poseFromOdom.getRotation());
    }


    public synchronized Optional<VisionUpdate> getLatestVisionUpdate() {
        return mLatestVisionUpdate;
    }

    public void outputTelemetry() {
        Logger.recordOutput("Robot Velocity", getMeasuredVelocity().toString());
        Logger.recordOutput("PoseFromOdometry",  getLatestPoseFromOdom().getValue().toWPI());
        Logger.recordOutput("Vision Pose Component", getAbsoluteVisionPoseComponent(Timer.getFPGATimestamp()).toWPI());
        Logger.recordOutput("Filtered Pose", getKalmanPose(Timer.getFPGATimestamp()).toWPI());
        Logger.recordOutput("SetPoint Pose", mSetpointPose.toWPI());
        Logger.recordOutput("Vision Pose", getDisplayVisionPose().toWPI());
        Logger.recordOutput("x hat", mKalmanFilter.getXhat(0));

   }

    public void setDisplaySetpointPose(Pose2d setpoint) {
        mSetpointPose = setpoint;
    }
}