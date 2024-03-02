// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.subsystems;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.annotation.JsonInclude.Include;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.Constants;
import com.uni.frc.Autos.Shoot;
import com.uni.frc.Constants.FieldConstants;
import com.uni.frc.Planners.AutoAlignPointSelector;
import com.uni.frc.Planners.DriveMotionPlanner;
import com.uni.frc.Planners.ShootingUtils;
import com.uni.frc.Planners.ShootingUtils.NoteState;
import com.uni.frc.Planners.ShootingUtils.ShootingParameters;
import com.uni.frc.subsystems.Pivot.State;
import com.uni.frc.subsystems.Requests.Request;
import com.uni.frc.subsystems.Requests.RequestList;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import com.uni.frc.subsystems.Vision.OdometryLimeLight;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Translation2d;
import com.uni.lib.motion.PathStateGenerator;
import com.uni.lib.swerve.ChassisSpeeds;
import com.uni.lib.util.InterpolatingDouble;
import com.uni.lib.util.InterpolatingTreeMap;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class SuperStructure extends Subsystem {
    protected Intake mIntake;
    protected OdometryLimeLight vision;
    protected DriveMotionPlanner mDriveMotionPlanner;
    protected Logger logger;
    protected DriveMotionPlanner dMotionPlanner;
    protected RobotState mRobotState;
    protected Indexer mIndexer;
    protected Pivot mPivot;
    protected Shooter mShooter;
    protected Wrist mWrist;
    protected Hand mHand;
    protected Arm mArm;
    protected SwerveDrive mDrive;

    private ArrayList<RequestList> queuedRequests;

    public SuperStructure() {
        vision = OdometryLimeLight.getInstance();
        mDriveMotionPlanner = DriveMotionPlanner.getInstance();
        mHand = Hand.getInstance();
        mArm = Arm.getInstance();
        mRobotState = RobotState.getInstance();
        mIntake = Intake.getInstance();
        mIndexer = Indexer.getInstance();
        mPivot = Pivot.getInstance();
        mShooter = Shooter.getInstance();
        mWrist = Wrist.getInstance();
        mDrive = SwerveDrive.getInstance();
        queuedRequests = new ArrayList<>();

    }

    public static SuperStructure instance = null;

    public static SuperStructure getInstance() {
        if (instance == null)
            instance = new SuperStructure();
        return instance;
    }

    private RequestList activeRequests;
    Request currentRequest;

    private boolean newRequests;
    private boolean activeRequestsComplete = true;
    private String currentRequestLog;
    private boolean stateChanged = false;
    private SuperState currentState = SuperState.OFF;
    private Mode currentMode = Mode.SHOOTING;
    private boolean modeChanged = false;
    private boolean allRequestsComplete;

    private boolean requestsCompleted() {
        return activeRequestsComplete;
    }

    boolean lockElevator = false;
    boolean cube = false;
    boolean isIntaking = false;

    private void setActiveRequests(RequestList requests) {
        activeRequests = requests;
        newRequests = true;
        activeRequestsComplete = false;
        allRequestsComplete = false;
    }

    public enum SuperState {
        INTAKING,
        CLIMB,
        SCORE,
        SOURCE,
        OFF,
        IDLE,
        AUTO,
        OUTTAKING
    }

    public enum Mode {
        SHOOTING,
        AMP
    }

    public void setState(SuperState state) {
        if (currentState != state)
            stateChanged = true;
        currentState = state;
    }

    public void setMode(Mode mode) {
        if (currentMode != mode)
            modeChanged = true;
        currentMode = mode;
    }

    private void setQueueRequests(RequestList r) {
        queuedRequests.clear();
        queuedRequests.add(r);
    }

    private void setQueueRequests(List<RequestList> requests) {
        queuedRequests.clear();
        queuedRequests = new ArrayList<>(requests.size());
        for (RequestList r : requests) {
            queuedRequests.add(r);
        }
    }

    private void request(Request r) {
        setActiveRequests(new RequestList(Arrays.asList(r), false));
        setQueueRequests(new RequestList());
    }

    private void request(Request r, Request q) {
        setActiveRequests(new RequestList(Arrays.asList(r), false));
        setQueueRequests(new RequestList(Arrays.asList(q), false));
    }

    private void request(RequestList r) {
        setActiveRequests(r);
        setQueueRequests(new RequestList());
    }

    private void request(RequestList r, RequestList q) {
        setActiveRequests(r);
        setQueueRequests(q);
    }

    public void addActiveRequests(Request r) {
        activeRequests.add(r);
        newRequests = true;
        activeRequestsComplete = false;
        allRequestsComplete = false;
    }

    public void addForemostActiveRequest(Request request) {
        activeRequests.addToForefront(request);
        newRequests = true;
        activeRequestsComplete = false;
        activeRequestsComplete = false;
    }

    public void queue(Request request) {
        queuedRequests.add(new RequestList(Arrays.asList(request), false));
    }

    public void queue(RequestList list) {
        queuedRequests.add(list);
    }

    public void replaceQueue(Request request) {
        setQueueRequests(new RequestList(Arrays.asList(request), false));
    }

    public synchronized void clearQueues() {
        queuedRequests.clear();
        activeRequests = new RequestList();
        activeRequestsComplete = true;
    }

    public void replaceQueue(RequestList list) {
        setQueueRequests(list);
    }

    public void replaceQueue(List<RequestList> lists) {
        setQueueRequests(lists);
    }

    public void processState(double timestamp) {
        Logger.recordOutput("current SuperState", currentState);
        switch (currentState) {
            case SCORE:
                switch (currentMode) {
                    case SHOOTING:
                        if (stateChanged) {
                            shootState(true);
                        }

                        prepareShooterSetpoints(timestamp);
                        mDrive.setState(SwerveDrive.State.AIMING);
                        break;

                    case AMP:
                        if (stateChanged) {
                            scoreAmpState();
                        }
                        Optional<Pose2d> targetSnap = AutoAlignPointSelector
                                .chooseTargetPoint(RobotState.getInstance().getKalmanPose(timestamp));
                        if (targetSnap.isEmpty()) {
                            mDrive.setState(SwerveDrive.State.MANUAL);

                        } else {
                            mDrive.setAlignment(targetSnap.get());
                            mDrive.setState(SwerveDrive.State.ALIGNMENT);
                        }
                        break;
                }

                break;

            case INTAKING:
                if (stateChanged && !DriverStation.isAutonomous()) {
                    intakeState(true);
                    mDrive.setState(SwerveDrive.State.TARGETOBJECT);
                }
                break;
            case OUTTAKING:
                mIntake.conformToState(Intake.State.OUTTAKING);
                break;
            case CLIMB:
                break;
            case SOURCE:
                mDrive.setState(SwerveDrive.State.ALIGNMENT);
                break;
            case IDLE:
                if (stateChanged)
                    clearQueues();

                mDrive.setState(SwerveDrive.State.MANUAL);
                mIntake.conformToState(Intake.State.OFF);
                mIndexer.conformToState(Indexer.State.OFF);
                if (mIndexer.hasPiece() && inZone(timestamp)) {
                    ShootingParameters shootingParameters = getShootingParams(mRobotState.getKalmanPose(timestamp));
                    mShooter.conformToState(Shooter.State.PARTIALRAMP);
                    mPivot.setMotionMagic(shootingParameters.uncompensatedDesiredPivotAngle);
                } else {
                    mShooter.conformToState(Shooter.State.IDLE);
                    mPivot.conformToState(Pivot.State.MAXDOWN);
                }
                if (modeChanged && currentMode == Mode.AMP) {
                    transferState(activeRequestsComplete);
                }
                break;
            case OFF:
                mPivot.stop();
                mDrive.stop();
                mIndexer.stop();
                mShooter.stop();
                mIntake.stop();
                mArm.stop();
                mHand.stop();
                break;
            case AUTO:
                break;

        }
        stateChanged = false;
        modeChanged = false;
    }

    public boolean inZone(double timestamp) {
        Pose2d currentPose = mRobotState.getKalmanPose(timestamp);
        if (DriverStation.getAlliance().get().equals(Alliance.Red))
            return currentPose.getTranslation().x() > 10.74;
        return currentPose.getTranslation().x() < 5.87;
    }

    public void update() {
        double timeStamp = Timer.getFPGATimestamp();
        synchronized (SuperStructure.this) {

            if (!activeRequestsComplete) {
                if (newRequests) {
                    if (activeRequests.isParallel()) {
                        boolean allActivated = true;
                        for (Iterator<Request> iterator = activeRequests.getRequests().iterator(); iterator
                                .hasNext();) {
                            Request request = iterator.next();
                            boolean allowed = request.allowed();
                            allActivated &= allowed;
                            if (allowed)
                                request.act();
                        }
                        newRequests = !allActivated;
                    } else {
                        if (activeRequests.isEmpty()) {
                            activeRequestsComplete = true;
                            return;
                        }
                        currentRequest = activeRequests.remove();
                        currentRequest.act();
                        currentRequest.initialize();
                        newRequests = false;
                    }
                }
                if (activeRequests.isParallel()) {
                    boolean done = true;
                    for (Request request : activeRequests.getRequests()) {
                        done &= request.isFinished();
                    }
                    activeRequestsComplete = done;
                } else if (currentRequest.isFinished()) {
                    if (activeRequests.isEmpty()) {
                        activeRequestsComplete = true;
                    } else if (activeRequests.getRequests().get(0).allowed()) {
                        newRequests = true;
                        activeRequestsComplete = false;
                    }
                } else {
                    currentRequest.act();
                }
            } else {
                if (!queuedRequests.isEmpty()) {
                    setActiveRequests(queuedRequests.remove(0));
                } else {
                    allRequestsComplete = true;
                }
            }

        }
        processState(timeStamp);
    }

    public boolean prepareShooterSetpoints(double timestamp) {
        ShootingParameters shootingParameters = getShootingParams(mRobotState.getKalmanPose(timestamp));
        Logger.recordOutput("desiredPivot", shootingParameters.compensatedDesiredPivotAngle);

        boolean allowShootWhileMove = true; // TODO
        if (allowShootWhileMove) {
            mPivot.conformToState(shootingParameters.compensatedDesiredPivotAngle);
            mShooter.conformToState(Shooter.State.SHOOTING);
        } else {
            mPivot.conformToState(shootingParameters.uncompensatedDesiredPivotAngle);
            mShooter.setPercent(shootingParameters.uncompensatedDesiredShooterSpeed);
        }
        mShooter.setSpin(shootingParameters.desiredSpin);
        mIndexer.setPiece(false);
        return ShootingUtils.pivotAtSetpoint(shootingParameters, Constants.PivotConstants.kDeadband,
                allowShootWhileMove)
                && ShootingUtils.shooterAtSetpoint(shootingParameters, Constants.ShooterConstants.kDeadband,
                        allowShootWhileMove);// TODO deadband
    }

    public void prepareShooterSetpoints() {
        ShootingParameters shootingParameters = getShootingParams(
                mRobotState.getPoseFromOdom(Timer.getFPGATimestamp()));
        if (currentState != SuperState.INTAKING)
            mPivot.conformToState(shootingParameters.compensatedDesiredPivotAngle);
        mShooter.conformToState(Shooter.State.SHOOTING);
        mShooter.setSpin(shootingParameters.desiredSpin);

    }

    private ShootingParameters getShootingParams(Pose2d currentPose) {
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shotTimeMap = Constants.ShooterConstants.SHOT_TRAVEL_TIME_TREE_MAP;
        double kShotTime = Constants.ShooterConstants.kShotTime;

        Pose2d speakerPose = Constants.getShooterPose();
        Pose2d robotToTarget = Pose2d
                .fromTranslation(speakerPose.getTranslation().translateBy(currentPose.getTranslation().inverse()));
        Logger.recordOutput("speakerPose", speakerPose.toWPI());
        Logger.recordOutput("Robotargo", robotToTarget.toWPI());

        double pivotAngle = 0; // mPivot.getAngle(); TODO
        double shooterVelovity = 0; // mShooter.getVelocity; TODO
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> pivotMap = ShootingUtils
                .getPivotMap(NoteState.NEW);

        ShootingParameters shootingParameters = ShootingUtils.getShootingParameters(
                robotToTarget,
                pivotAngle,
                shooterVelovity,
                kShotTime,
                pivotMap,
                shotTimeMap,
                mRobotState.getPredictedVelocity());// TODO change to measured when it works
        return shootingParameters;
    }

    public void intakePercent(double percentage) {
        RequestList request = new RequestList(Arrays.asList(
                mIntake.setIntakePercentRequest(percentage)), false);

        request(request);
    }

    public void trajectoryState(PathPlannerTrajectory trajectory, double initRotation) {
        RequestList request = new RequestList(Arrays.asList(
                logCurrentRequest("trajectory")), true);
        RequestList queue = new RequestList(Arrays.asList(
                mDriveMotionPlanner.setTrajectoryRequest(trajectory, initRotation, true),
                mDrive.setStateRequest(SwerveDrive.State.TRAJECTORY),
                mDriveMotionPlanner.startPathRequest(true)), false);
        queue(request);
        queue(queue);
    }

    public void onTheFlyTrajectoryState(Pose2d endPose, double timestamp) {
        RequestList request = new RequestList(Arrays.asList(
                logCurrentRequest("OnTheFly"),
                mDriveMotionPlanner.generatePathRequest(
                        mRobotState.getKalmanPose(timestamp),
                        endPose,
                        ChassisSpeeds.fromTwist2d(mRobotState.getPredictedVelocity(), mDrive.getRobotHeading()),
                        false),
                mDrive.setStateRequest(SwerveDrive.State.TRAJECTORY),
                mDriveMotionPlanner.startPathRequest(false)), false);

        request(request);
    }

    // public void intakeState(boolean Override,double firingPositionTime){
    // intakeState(Override,
    // PathStateGenerator.getInstance().getDesiredPose2d(false,firingPositionTime));
    // }

    // public void intakeState(boolean Override,Pose2d firingPosition){

    // RequestList request = new RequestList(Arrays.asList(
    // logCurrentRequest("Intaking"),
    // mIntake.stateRequest(Intake.State.Intaking),
    // mIntake.hasPieceRequest()
    // ), true);
    // RequestList queue = new RequestList(Arrays.asList(
    // logCurrentRequest("Transfering"),
    // mIntake.hasPieceRequest(),
    // mIntake.stateRequest(Intake.State.Feeding),
    // mPivot.stateRequest(getShootingParams(firingPosition).uncompensatedDesiredPivotAngle),
    // mShooter.setPercentRequest(getShootingParams(firingPosition).uncompensatedDesiredShooterSpeed)
    // ), false);

    // if(Override){
    // request(request,queue);
    // }
    // else{
    // queue(request);
    // queue(queue);
    // }
    // }
    public void intakeState(boolean Override) {

        if (Override) {
            RequestList request = new RequestList(Arrays.asList(
                    logCurrentRequest("Intaking"),
                    mPivot.stateRequest(-.226),
                    mPivot.atTargetRequest(),
                    mIntake.stateRequest(Intake.State.INTAKING),
                    mIndexer.stateRequest(Indexer.State.RECIEVING),
                    mIndexer.hasPieceRequest(!Override),
                    waitRequest(0),
                    mIndexer.stateRequest(Indexer.State.OFF),
                    mIntake.stateRequest(Intake.State.OFF)), false);
            request(request);
        } else {
            RequestList request = new RequestList(Arrays.asList(
                    logCurrentRequest("Intaking"),
                    setStateRequest(SuperState.INTAKING),
                    mPivot.stateRequest(-.226),
                    mIntake.stateRequest(Intake.State.INTAKING),
                    mIndexer.stateRequest(Indexer.State.RECIEVING),
                    mIndexer.hasPieceRequest(true),
                    setStateRequest(SuperState.AUTO),
                    mIndexer.stateRequest(Indexer.State.OFF),
                    mIntake.stateRequest(Intake.State.OFF)), false);

            queue(request);
        }

    }

    public void intakeState(double timeout) {
        RequestList request = new RequestList(Arrays.asList(
                logCurrentRequest("Intaking"),
                setStateRequest(SuperState.INTAKING),
                mPivot.stateRequest(-.226),
                // mIntake.stateRequest(Intake.State.INTAKING),
                mIndexer.stateRequest(Indexer.State.RECIEVING),
                waitRequest(.3),
                // mIndexer.hasPieceRequest(timeout),
                setStateRequest(SuperState.AUTO),
                mIndexer.stateRequest(Indexer.State.OFF)),false);
                // mIntake.stateRequest(Intake.State.OFF)), false);
        queue(request);

    }

    public void waitForEventState(double timestamp) {
        queue(mDriveMotionPlanner.waitForTrajectoryRequest(timestamp));
    }

    public void waitForPositionState(Translation2d other) {
        queue(
                new Request() {
                    @Override
                    public boolean isFinished() {
                        Logger.recordOutput("event", Pose2d.fromTranslation(other.reflect()).toWPI());

                        if (DriverStation.getAlliance().get().equals(Alliance.Blue))
                            return other.translateBy(
                                    mRobotState.getPoseFromOdom(Timer.getFPGATimestamp()).getTranslation().inverse())
                                    .norm() < .3;
                        return other.reflect().translateBy(
                                mRobotState.getPoseFromOdom(Timer.getFPGATimestamp()).getTranslation().inverse())
                                .norm() < .3;

                    }
                });
    }

    public void waitForPositionState(double time) {
        Translation2d other = mDriveMotionPlanner.sample(time).getTranslation();
        new Request() {
            @Override
            public boolean isFinished() {
                if (DriverStation.getAlliance().get().equals(Alliance.Blue))
                    return other.translateBy(
                            mRobotState.getPoseFromOdom(Timer.getFPGATimestamp()).getTranslation().inverse())
                            .norm() < .3;
                return other.reflect().translateBy(
                        mRobotState.getPoseFromOdom(Timer.getFPGATimestamp()).getTranslation().inverse())
                        .norm() < .3;
            }
        };

    }

    public void shootState(boolean Override) {
        if (Override) {
            RequestList queue = new RequestList(Arrays.asList(
                    mShooter.atTargetRequest(),
                    mPivot.atTargetRequest(),
                    mDrive.isAimedRequest(),
                    mIntake.stateRequest(Intake.State.INTAKING),
                    mIndexer.stateRequest(Indexer.State.TRANSFERING),
                    waitRequest(.3),
                    mShooter.stateRequest(Shooter.State.IDLE),
                    mIndexer.setHasPieceRequest(false)), false);
            request(queue);
        } else {
            RequestList queue = new RequestList(Arrays.asList(
                    mPivot.atTargetRequest(),
                    // mShooter.atTargetRequest(.5),
                    mIndexer.stateRequest(Indexer.State.TRANSFERING),
                    waitRequest(.3),
                    mIndexer.stateRequest(Indexer.State.OFF),
                    mIndexer.setHasPieceRequest(false)), false);
            queue(queue);
        }
    }

    public Request logCurrentRequest(String newLog) {
        return new Request() {
            @Override
            public void act() {
                currentRequestLog = newLog;
            }
        };
    }

    public String getCurrentLoggedRequest() {
        return currentRequestLog;
    }

    public void transferState(boolean Override) {
        RequestList request = new RequestList(Arrays.asList(

                mPivot.stateRequest(Pivot.State.TRANSFER),
                mWrist.stateRequest(Wrist.State.TRANSFER),
                mArm.stateRequest(Arm.State.TRANSFER),

                mPivot.atTargetRequest(),
                mWrist.atTargetRequest(),
                mArm.atTargetRequest(),

                mIndexer.stateRequest(Indexer.State.TRANSFERING),
                mShooter.stateRequest(Shooter.State.TRANSFER),
                mHand.stateRequest(Hand.State.TRANSFERING),

                mHand.hasPieceRequest(true),

                mHand.stateRequest(Hand.State.OFF),
                mWrist.stateRequest(Wrist.State.SHOOTING),
                mShooter.stateRequest(Shooter.State.IDLE),
                mIndexer.stateRequest(Indexer.State.OFF),
                mPivot.stateRequest(Pivot.State.MAXDOWN)),
                false);
        if (Override)
            request(request);
        else
            queue(request);
    }

    public void scoreAmpState() {
        RequestList request = new RequestList(Arrays.asList(
                mPivot.stateRequest(Pivot.State.AMP),
                mArm.stateRequest(Arm.State.AMP),
                mWrist.stateRequest(Wrist.State.AMP),

                mPivot.atTargetRequest(),
                mArm.atTargetRequest(),
                mWrist.atTargetRequest(),

                mHand.stateRequest(Hand.State.SHOOTING),
                waitRequest(.2),
                mHand.stateRequest(Hand.State.OFF),

                mPivot.stateRequest(Pivot.State.MAXDOWN),
                mArm.stateRequest(Arm.State.MAXDOWN),
                mWrist.stateRequest(Wrist.State.AMP)

        ), false);
        if (!mHand.hasPiece() && mIndexer.hasPiece()) {
            transferState(true);
            queue(request);
        } else {
            request(request);
        }

    }

    public void reverseTransferState() {
        RequestList request = new RequestList(Arrays.asList(

                mPivot.stateRequest(Pivot.State.TRANSFER),
                mWrist.stateRequest(Wrist.State.TRANSFER),
                mArm.stateRequest(Arm.State.TRANSFER),

                mPivot.atTargetRequest(),
                mWrist.atTargetRequest(),
                mArm.atTargetRequest(),

                mIndexer.stateRequest(Indexer.State.REVERSE_TRANSFER),
                mShooter.stateRequest(Shooter.State.REVERSETRANSFER),
                mHand.stateRequest(Hand.State.REVERSETRANSFER),

                mIndexer.hasPieceRequest(true),

                mHand.stateRequest(Hand.State.OFF),
                mWrist.stateRequest(Wrist.State.SHOOTING),
                mShooter.stateRequest(Shooter.State.IDLE),
                mIndexer.stateRequest(Indexer.State.OFF),
                mPivot.stateRequest(Pivot.State.MAXDOWN),
                mArm.stateRequest(Arm.State.MAXDOWN)),
                false);
        request(request);
    }

    public void waitState(double waitTime, boolean Override) {
        if (Override)
            request(waitRequest(waitTime));
        else
            queue(waitRequest(waitTime));
    }

    public Request waitRequest(double waitTime) {
        return new Request() {
            Timer timer = new Timer();

            @Override
            public boolean isFinished() {
                if (timer.hasElapsed(waitTime)) {
                    timer.reset();
                    return true;
                } else
                    return timer.hasElapsed(waitTime);
            }

            @Override
            public void act() {
                timer.start();
            }
        };
    }

    public Request setStateRequest(SuperState state) {
        return new Request() {
            public void act() {
                setState(state);
            }
        };
    }

    public Request neverRequest() {
        return new Request() {
            @Override
            public boolean isFinished() {

                return false;
            }

            @Override
            public void act() {
            }
        };
    }

    @Override
    public void outputTelemetry() {
        Logger.recordOutput("RequestsCompleted", requestsCompleted());
        Logger.recordOutput("CurrentRequest", currentRequestLog);
    }

    @Override
    public void stop() {

    }
}
