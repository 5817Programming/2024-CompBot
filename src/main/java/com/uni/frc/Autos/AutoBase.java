// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.Autos;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathSegment;
import com.uni.frc.subsystems.RobotState;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import com.uni.lib.EventMarker;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Translation2d;
import com.uni.lib.motion.PathStateGenerator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;

/** 
 * This is an abstract class
 * abstract classes tell its child methods it needs to have or it will raise an errror
 * we have to implement a method called auto
 * this class will also give its child the stop auto method because all autos will need it
 */
public abstract class AutoBase {
    public PathStateGenerator mPathStateGenerator;
    public RobotState mRobotState;
    public SuperStructure s;
    public List<Pose2d> stopPoses = new ArrayList<>();
    public AutoBase(){
        mPathStateGenerator = PathStateGenerator.getInstance();
        mRobotState = RobotState.getInstance();
        s = SuperStructure.getInstance();
    }
    public abstract void auto();

    public void runAuto(){
        auto();
    }

    public void stopAuto(){
        SuperStructure.getInstance().clearQueues();
        SwerveDrive.getInstance().setState(SwerveDrive.State.MANUAL);
    }

    public void registerTrajectoryStops(List<Double> stopTimeStamps){
        for(double m: stopTimeStamps){
            stopPoses.add(mPathStateGenerator.sample(m));
        }
    }


    public void registerTrajectoryEvents(String pathName){
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        List<EventMarker> eventMarkers = new ArrayList<>();
        var allPoints = path.getAllPathPoints();
        try (BufferedReader br =
        new BufferedReader(
            new FileReader(
                new File(
                    Filesystem.getDeployDirectory(), "pathplanner/paths/" + pathName + ".path")))) {
      StringBuilder fileContentBuilder = new StringBuilder();
      String line;
      while ((line = br.readLine()) != null) {
        fileContentBuilder.append(line);
      }

      String fileContent = fileContentBuilder.toString();
      JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

    for (var markerJson : (JSONArray) json.get("eventMarkers")) {
      eventMarkers.add(EventMarker.fromJson((JSONObject) markerJson));
    }
    for (EventMarker m : eventMarkers) {
        int pointIndex = (int) Math.round(m.getWaypointRelativePos() / PathSegment.RESOLUTION);
        m.markerPos = new Translation2d(allPoints.get(pointIndex).position);
        Logger.recordOutput("Marker Pose " + eventMarkers.indexOf(m), Pose2d.fromTranslation(m.markerPos).toWPI());

        if(m.getName().equals("Shoot")){
            s.waitForPositionState(m.markerPos);
            s.printState("Intaking + Shooting");
            s.shootState(false);
            s.resumeTrajectoryState();
            stopPoses.add(Pose2d.fromTranslation(new Translation2d(m.markerPos)));
        }
        else if(m.getName().equals("Shoot Intake")){
            s.waitForPositionState(m.markerPos);
            s.printState("Intaking + Shooting");
            s.intakeState(.7, false);
        }
        else if(m.getName().equals("Intake")){
            s.waitForPositionState(m.markerPos);
            s.setContinuousShootState(false);
            s.printState("Intaking");
            s.intakeState(.7, false);
            s.setContinuousShootState(true);

        }
        else{
            System.out.println("Invalid event name: "+ m.getName());
        }
    }
    } catch (Exception e) {
      e.printStackTrace();
    }
        
    
    }

    public void updateAuto(double timestamp){
        if(!stopPoses.isEmpty())
            for(int i = 0; i < stopPoses.size(); i++){
                if (DriverStation.getAlliance().get().equals(Alliance.Blue)){
                    if(stopPoses.get(i).getTranslation().translateBy(mRobotState.getKalmanPose(timestamp).getTranslation().inverse()).norm() < .3){
                        mPathStateGenerator.stopTimer();
                        stopPoses.remove(i);
                    }
                }else{
                    if(stopPoses.get(i).getTranslation().reflect().translateBy(mRobotState.getKalmanPose(timestamp).getTranslation().inverse()).norm() < .3){
                        mPathStateGenerator.stopTimer();
                        stopPoses.remove(i);
                    }
                }
            }
    }

}
