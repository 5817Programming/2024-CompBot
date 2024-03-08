// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.Autos;

import java.util.ArrayList;
import java.util.List;

import com.uni.frc.Planners.DriveMotionPlanner;
import com.uni.frc.subsystems.RobotState;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.motion.PathStateGenerator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

/** 
 * This is an abstract class
 * abstract classes tell its child methods it needs to have or it will raise an errror
 * we have to implement a method called auto
 * this class will also give its child the stop auto method because all autos will need it
 */
public abstract class AutoBase {
    public PathStateGenerator mPathStateGenerator;
    public RobotState mRobotState;
    public List<Pose2d> stopPoses = new ArrayList<>();
    public AutoBase(){
        mPathStateGenerator = PathStateGenerator.getInstance();
        mRobotState = RobotState.getInstance();
    }
    public abstract void auto();
    public void testAuto(){

    }

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
