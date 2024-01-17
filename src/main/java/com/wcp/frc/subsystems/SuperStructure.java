// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.wcp.frc.Constants;
import com.wcp.frc.subsystems.Requests.Request;
import com.wcp.frc.subsystems.Requests.RequestList;
import com.wcp.lib.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class SuperStructure extends Subsystem {
    public Swerve swerve;
    public Intake intake;
    public Vision vision;
    public Logger logger;

    private ArrayList<RequestList> queuedRequests;

    public SuperStructure() {
        swerve = Swerve.getInstance();
        vision = Vision.getInstance();
        intake = Intake.getInstance();
        logger = Logger.getInstance();
        idleState();
        queuedRequests = new ArrayList<>();

    }

    public static SuperStructure instance = null;

    public static SuperStructure getInstance() {
        if (instance == null)
            instance = new SuperStructure();
        return instance;
    }

    private RequestList activeRequests;
    private RequestList idleRequests;
    private Translation2d swerveControls = new Translation2d();
    private double swerveRotation = 0;
    Request currentRequest;

    private boolean newRequests;
    private boolean activeRequestsComplete = true;
    private String currentRequestLog;
    private boolean allRequestsComplete;

    private boolean requestsCompleted() {
        return activeRequestsComplete;
    }


    PreState currentUnprocessedState = PreState.ZERO;

    boolean lockElevator = false;
    boolean cube = false;
    boolean isIntaking = false;

    public enum GameState {
        GETPIECE,
        SCORE,
        CHARGE;
    }

    public enum PreState {
        HIGH,
        MID,
        LOW,
        HOOMAN,
        CHUTE,
        ZERO,
        GROUND;

    }



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

    public void requestSwerveInput(Translation2d x, double r) {
        this.swerveControls = x;
        this.swerveRotation = r;
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
                    idleState();
                    for (Iterator<Request> iterator = idleRequests.getRequests().iterator(); iterator.hasNext();) {
                        Request request = iterator.next();
                        boolean allowed = request.allowed();
                        if (allowed)
                            request.act();
                    }
                    activeRequestsComplete = true;
                }
            }

        }
    }

    public void objectTargetState(boolean fixedRotation) {
        RequestList request = new RequestList(Arrays.asList(
                logCurrentRequest("objectTarget"),
                swerve.objectTargetRequest(fixedRotation)
                ), false);
        queue(request);
    }


    public void idleState() {
        currentRequestLog = "idle";
        RequestList request = new RequestList(Arrays.asList(
                swerve.openLoopRequest(swerveControls, swerveRotation),
                vision.pipleLineRequest(Constants.VisionConstants.APRIL_PIPLINE)), true);
        idleRequests = request;
    }

    public void intakePercent(double percentage) {
        RequestList request = new RequestList(Arrays.asList(
            intake.setIntakePercentRequest(percentage)
                ), true);

        queue(request);
    }

    public void trajectoryState(PathPlannerTrajectory trajectory) {
        RequestList request = new RequestList(Arrays.asList(
                logCurrentRequest("trajectory")
        ), true);
        RequestList queue = new RequestList(Arrays.asList(
                swerve.setTrajectoryRequest(trajectory),
                swerve.startPathRequest(true)), false);
        queue(request);
        queue(queue);
    }

    public void waitForTrajectoryState(double PercentageToRun) {
        RequestList request = new RequestList(Arrays.asList(
                swerve.waitForTrajectoryRequest(PercentageToRun)),
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
        logger.recordOutput("RequestsCompleted", requestsCompleted());
        logger.recordOutput("CurrentRequest", currentRequestLog);
    }

    @Override
    public void stop() {

    }
}
