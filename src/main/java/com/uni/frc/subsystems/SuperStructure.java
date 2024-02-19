// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.Constants;
import com.uni.frc.Planners.DriveMotionPlanner;
import com.uni.frc.Planners.ShootingUtils;
import com.uni.frc.Planners.ShootingUtils.NoteState;
import com.uni.frc.Planners.ShootingUtils.ShootingParameters;
import com.uni.frc.subsystems.Requests.Request;
import com.uni.frc.subsystems.Requests.RequestList;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import com.uni.frc.subsystems.Vision.OdometryLimeLight;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.swerve.ChassisSpeeds;
import com.uni.lib.util.InterpolatingDouble;
import com.uni.lib.util.InterpolatingTreeMap;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class SuperStructure extends Subsystem {
    public SwerveDrive swerve;
    public Intake mIntake;
    public OdometryLimeLight vision;
    public DriveMotionPlanner driveMotionPlanner;
    public Logger logger;
    public DriveMotionPlanner dMotionPlanner;
    public RobotState mRobotState;
    public Indexer mIndexer;
    public Pivot mPivot;
    public Shooter mShooter;
    public Wrist mWrist;
    public Hand mHand;

    private ArrayList<RequestList> queuedRequests;

    public SuperStructure() {
        swerve = SwerveDrive.getInstance();
        vision = OdometryLimeLight.getInstance();
        driveMotionPlanner = DriveMotionPlanner.getInstance();
        mRobotState = RobotState.getInstance();
        mIntake = Intake.getInstance();
        mIndexer = Indexer.getInstance();
        mPivot = Pivot.getInstance();
        mShooter = Shooter.getInstance();
        mWrist = Wrist.getInstance();
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
    private boolean hasPiece = false;
    private SuperState currentState = SuperState.OFF;
    private SuperState desiredState = SuperState.OFF;
    private ShootingState shootingState = ShootingState.OFF;
    private ShootingState desiredShootingState = ShootingState.OFF;
    private ClimbState climbState = ClimbState.OFF;
    private ClimbState desiredClimbState = ClimbState.OFF;

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

    private enum SuperState{
        INTAKING,
        SHOOTING,
        CLIMB,
        AMP,
        SOURCE,
        OFF
    }

    private enum ShootingState{
        OFF,
        PREPARED,
        RAMPING,
        TRANSFERING,
        SHOOTING,
    }
    private enum ClimbState{
        OFFCHAIN,
        ONCHAIN,
        TRAPPING,
        OFF,
    }
    private void setState(SuperState state){
        if(currentState != state)
            stateChanged = true;
        currentState = state;
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
    public void setDesiredState(SuperState state){
        desiredState = state;
    }
    public void processShootingState(double timestamp){
        
        switch (shootingState){
            case RAMPING:
                if(prepareShooterSetpoints(timestamp)){
                    shootingState = ShootingState.PREPARED;
                }
                break;
            case SHOOTING:

                mIndexer.setPercent(1);
                break;
            case OFF:
                mIndexer.setPercent(0);
                mShooter.setPercent(0);
                break;
            case TRANSFERING:

                transferState();

                
                
                break;
            case PREPARED:
                if(readyToShoot()){
                    shootingState = ShootingState.SHOOTING;
                }

                break;
        }
    }
    public void processClimbState(double timestamp){
        
        switch (climbState) {//TODO handle states
            case TRAPPING:

                break;
            case OFFCHAIN:
                break;
            case ONCHAIN:
                break;
            case OFF:
                break;
            
        }
    stateChanged = false;

    }

    public boolean readyToShoot(){
        return true;
    }
    public void processState(double timestamp){
        if(desiredState != currentState){
            switch (desiredState) {
                case SHOOTING: 
                    shootingState = ShootingState.RAMPING;
                    break;
                case AMP:
                    shootingState = ShootingState.TRANSFERING;
                case INTAKING:
                    break;
                case CLIMB:
                    break;
                case SOURCE:
                    break;
                case OFF:
                    mIntake.setState(Intake.State.Off);
                    shootingState = ShootingState.OFF;
                    climbState = ClimbState.OFF;
                    mHand.stateRequest(Hand.State.OFF);
                    break;
     

            }
        }
        processShootingState(timestamp);
        stateChanged = false;
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
                    activeRequestsComplete = true;

                }
            }

        }
        // processState(timeStamp);
    }


    public boolean prepareShooterSetpoints(double timestamp){
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shotTimeMap = Constants.ShooterConstants.SHOT_TRAVEL_TIME_TREE_MAP;
        double kShotTime = Constants.ShooterConstants.kShotTime;

        Pose2d speakerPose = Constants.getShooterPose();
        Pose2d currentPose = mRobotState.getKalmanPose(timestamp);
        Pose2d robotToTarget = currentPose.transformBy(speakerPose);

        boolean allowShootWhileMove = true; //TODO
        double pivotAngle = 0; // mPivot.getAngle(); TODO
        double shooterVelovity = 0; //mShooter.getVelocity; TODO
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> pivotMap = ShootingUtils.getPivotMap(NoteState.NEW);
    
        ShootingParameters shootingParameters = ShootingUtils.getShootingParameters(
            robotToTarget, 
            pivotAngle, 
            shooterVelovity, 
            kShotTime, 
            pivotMap,
            shotTimeMap,
            mRobotState.getPredictedVelocity());//TODO change to measured when it works
        
        if(allowShootWhileMove){
            mPivot.conformToState(shootingParameters.compensatedDesiredPivotAngle);
            mShooter.setPercent(shootingParameters.compensatedDesiredShooterSpeed);
        }else{
             mPivot.conformToState(shootingParameters.uncompensatedDesiredPivotAngle);
            mShooter.setPercent(shootingParameters.uncompensatedDesiredShooterSpeed);
        }
        mShooter.setSpin(shootingParameters.desiredSpin);


        return ShootingUtils.pivotAtSetpoint(shootingParameters, Constants.PivotConstants.kDeadband, allowShootWhileMove)
                && ShootingUtils.shooterAtSetpoint(shootingParameters, Constants.ShooterConstants.kDeadband, allowShootWhileMove);//TODO deadband
    }




  
    public void dynamicScoreState(boolean Override){
        RequestList request = new RequestList(Arrays.asList(
            logCurrentRequest("Aiming"),
            vision.hasTargetRequest()
            // pivot.stateRequest(vision.getPivotAngle()),
            // shooter.stateRequest(Shooter.State.Ramping),
            // pivot.atTargetRequest()
            // shooter.atTargetRequest()
        ),false);

        queue(request);
    }


    public void intakePercent(double percentage) {
        RequestList request = new RequestList(Arrays.asList(
            mIntake.setIntakePercentRequest(percentage)
        ), false);

        request(request);
    }

    public void trajectoryState(PathPlannerTrajectory trajectory, double nodes,double initRotation) { 
        RequestList request = new RequestList(Arrays.asList(
                logCurrentRequest("trajectory")
        ), true);
        RequestList queue = new RequestList(Arrays.asList(
                driveMotionPlanner.setTrajectoryRequest(trajectory, nodes,initRotation, true),
                swerve.setStateRequest(SwerveDrive.State.TRAJECTORY),
                driveMotionPlanner.startPathRequest(true)), false);
        queue(request);
        queue(queue);
    } 

    public void onTheFlyTrajectoryState(Pose2d endPose, double timestamp){
        RequestList request = new RequestList(Arrays.asList(
            logCurrentRequest("OnTheFly"),
            driveMotionPlanner.generatePathRequest(
                mRobotState.getKalmanPose(timestamp),
                endPose,
                ChassisSpeeds.fromTwist2d(mRobotState.getPredictedVelocity(), swerve.getRobotHeading()),
                false),
            swerve.setStateRequest(SwerveDrive.State.TRAJECTORY),
            driveMotionPlanner.startPathRequest(false)
        ), false);

        request(request);
    }
    public void intakeState(boolean Override){
        RequestList request = new RequestList(Arrays.asList(
            logCurrentRequest("Intaking"),
            mIntake.stateRequest(Intake.State.Intaking),
            mIntake.hasPieceRequest()
        ), true);
        RequestList queue = new RequestList(Arrays.asList(
            logCurrentRequest("Transfering"),
            mIntake.hasPieceRequest(),
            mIntake.stateRequest(Intake.State.Feeding),
            mIndexer.stateRequest(Indexer.State.RECIEVING),
            mIndexer.hasPieceRequest()

        ), false);

        if(Override){
            request(request,queue);
        }
        else{
            queue(request);
            queue(queue);
        }
    }

    public void waitForTrajectoryState(double TimeToElapse) { 
        RequestList request = new RequestList(Arrays.asList(
                driveMotionPlanner.waitForTrajectoryRequest(TimeToElapse)),
                false);
        queue(request);
    }


    public Request logCurrentRequest(String newLog) {
        return new Request() {
            @Override
            public void act() {
                currentRequestLog = newLog;

            }
        };
    }
    public void transferState(){
        RequestList request = new RequestList(Arrays.asList(
        mIndexer.setPercentRequest(0.5),
        mShooter.setPercentRequest(0.5),
        mHand.stateRequest(Hand.State.TRANSFERING),
        mHand.hasPieceRequest(),
        mHand.stateRequest(Hand.State.OFF),
        mShooter.setPercentRequest(0),
        mIndexer.setPercentRequest(0)
        ),
        false);
            queue(request);  
        }
    public void waitState(double waitTime, boolean Override){
        if(Override)
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
