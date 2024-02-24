// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.Controls;

import java.util.Optional;

import com.uni.frc.Ports;
import com.uni.frc.Planners.AutoAlignPointSelector;
import com.uni.frc.subsystems.Indexer;
import com.uni.frc.subsystems.Intake;
import com.uni.frc.subsystems.Pivot;
import com.uni.frc.subsystems.RobotState;
import com.uni.frc.subsystems.Shooter;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.SuperStructure.SuperState;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import com.uni.frc.subsystems.Swerve.SwerveDrive.State;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.Timer;

public class Controls {
    SuperStructure s;
    Controller Driver;
    Controller CoDriver;
    SwerveDrive swerve;

    private static Controls instance = null;

    public static Controls getInstance() {
        if (instance == null)
            instance = new Controls();
        return instance;
    }

    public Controls() {
        Driver = new Controller(Ports.XBOX_1);
        CoDriver = new Controller(Ports.XBOX_2);
        swerve = SwerveDrive.getInstance();
    }
    public enum scoreMode{
        AMP,
        SPEAKER
    }
    scoreMode currentScoreMode = scoreMode.SPEAKER;
double percent = 0;
    public void update() {
        Driver.update();
        CoDriver.update();
        s = SuperStructure.getInstance();

        
        // if(Driver.RightBumper.isActive()){
        //     s.intakePercent(-percent);
        // }
        // else{
        //     s.intakePercent(0);
        // }

        if (Driver.StartButton.isPressed())
            swerve.resetGryo(180);
        if (Driver.LeftTrigger.getValue() > 0.2) {
            switch (currentScoreMode) {
                case SPEAKER:
                    s.setState(SuperState.SHOOTING);
                    break;
            
                case AMP:
                    s.setState(SuperState.AMP);
                    break;
        } }

           
            
        
        // else if(Driver.BButton.isPressed()){
        //     s.onTheFlyTrajectoryState(new Pose2d(8,2, Rotation2d.fromDegrees(180)), timestamp);

        
        if(Driver.RightTrigger.getValue()>0.2){ 
            s.setState(SuperState.INTAKING);
        }else if(Driver.RightBumper.isActive()){
            s.setState(SuperState.SHOOTING);
        } 
        else{
            s.setState(SuperState.IDLE);
        }
        swerve.sendInput(-Driver.LeftStickY.getValue(), Driver.LeftStickX.getValue(), Driver.RightStickX.getValue());
    }}


