// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.Controls;
import com.uni.frc.Ports;
import com.uni.frc.subsystems.Climb;
import com.uni.frc.subsystems.Indexer;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.SuperStructure.Mode;
import com.uni.frc.subsystems.SuperStructure.SuperState;
import com.uni.frc.subsystems.Swerve.SwerveDrive;


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

    double percent = 0;
    public void update() {
        Driver.update();
        CoDriver.update();
        s = SuperStructure.getInstance();
        Driver.rumble(Indexer.getInstance().hasPiece());

        if(CoDriver.DpadUp.isPressed())
            s.offsetPivot(0.002);
        if(CoDriver.DpadDown.isPressed())
            s.offsetPivot(-0.002);

        // if(Driver.RightBumper.isActive()){
        //     s.intakePercent(-percent);
        // }
        // else{
        //     s.intakePercent(0);
        // }
        s.setManual(CoDriver.BButton.isActive());
        if (Driver.StartButton.isPressed())
            swerve.fieldzeroSwerve();

        if(Driver.AButton.isActive()||CoDriver.AButton.isActive())
            Climb.getInstance().conformToState(Climb.State.UP);
        else 
            Climb.getInstance().conformToState(Climb.State.Down);
        // else if(Driver.BButton.isPressed()){
        //     s.onTheFlyTrajectoryState(new Pose2d(8,2, Rotation2d.fromDegrees(180)), timestamp);
        if(Driver.LeftBumper.isActive())
            s.setMode(Mode.FIRING);
        else
            s.setMode(Mode.SHOOTING);
        if(Driver.LeftTrigger.value >0.2){
            s.setState(SuperState.OUTTAKING);    

        }
        else if(Driver.RightTrigger.getValue()>0.2){ 
            s.setState(SuperState.INTAKING);
        }else if(Driver.RightBumper.isActive()){
            s.setState(SuperState.SCORE);
        } 
        else{
            s.setState(SuperState.IDLE);
        }
        swerve.sendInput(-Driver.LeftStickY.getValue(), Driver.LeftStickX.getValue(), Driver.RightStickX.getValue());
    }}


