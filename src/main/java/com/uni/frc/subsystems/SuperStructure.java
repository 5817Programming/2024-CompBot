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
import com.uni.frc.subsystems.Requests.Request;
import com.uni.frc.subsystems.Requests.RequestList;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import com.uni.frc.subsystems.Swerve.SwerveDrive.State;
import com.uni.frc.subsystems.Vision.OdometryLimeLight;
import com.uni.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class SuperStructure extends Subsystem {
    public SwerveDrive swerve;
    // public Intake intake;
    public OdometryLimeLight vision;
    public DriveMotionPlanner driveMotionPlanner;
    public Logger logger;
    public DriveMotionPlanner dMotionPlanner;
    // public Indexer indexer;
    // public Pivot pivot;
    // public Shooter shooter;

    private ArrayList<RequestList> queuedRequests;

    public SuperStructure() {
        swerve = SwerveDrive.getInstance();
        vision = OdometryLimeLight.getInstance();
        driveMotionPlanner = DriveMotionPlanner.getInstance();
        // intake = Intake.getInstance();
        // indexer = Indexer.getInstance();
        // pivot = Pivot.getInstance();
        // shooter = Shooter.getInstance();
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

  

  
    public void update() {
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
            // intake.setIntakePercentRequest(percentage)
        ), true);

        queue(request);
    }

    public void trajectoryState(PathPlannerTrajectory trajectory, double nodes,double initRotation) { 
        RequestList request = new RequestList(Arrays.asList(
                logCurrentRequest("trajectory")
        ), true);
        RequestList queue = new RequestList(Arrays.asList(
                driveMotionPlanner.setTrajectoryRequest(trajectory, nodes,initRotation),
                swerve.setStateRequest(State.TRAJECTORY),
                driveMotionPlanner.startPathRequest(true)), false);
        queue(request);
        queue(queue);
    } 

    public void intakeState(boolean Override){
        RequestList request = new RequestList(Arrays.asList(
            logCurrentRequest("Intaking")
            // intake.stateRequest(Intake.State.Intaking),
            // intake.hasPieceRequest()
        ), true);
        RequestList queue = new RequestList(Arrays.asList(
            logCurrentRequest("Transfering")
            // intake.hasPieceRequest(),
            // intake.stateRequest(Intake.State.Feeding),
            // indexer.stateRequest(Indexer.State.Recieving),
            // indexer.hasPieceRequest()
        ), false);

        if(Override){
            request(request,queue);
        }
        else{
            queue(request);
            queue(queue);
        }
    }

    public void waitForTrajectoryState(double PercentageToRun) { 
        RequestList request = new RequestList(Arrays.asList(
                driveMotionPlanner.waitForTrajectoryRequest(PercentageToRun)),
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
