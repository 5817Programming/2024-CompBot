// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.uni.frc.Planners.AutoAlignPointSelector;
import com.uni.frc.subsystems.RobotState;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import com.uni.frc.subsystems.Swerve.SwerveDrive.State;
import com.uni.lib.geometry.Pose2d;

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
double percent = 0;
    public void update() {
        Driver.update();
        CoDriver.update();
        var timestamp = Timer.getFPGATimestamp();
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
            swerve.sendInput(-Driver.LeftStickY.getValue(), Driver.LeftStickX.getValue(), -Driver.RightStickX.getValue(), State.AIMING);
        } else if (Driver.RightTrigger.getValue() > .2) {
            Optional<Pose2d> targetSnap = AutoAlignPointSelector
                    .chooseTargetPoint(RobotState.getInstance().getKalmanPose(timestamp));
            if (targetSnap.isEmpty()) {
            swerve.sendInput(-Driver.LeftStickY.getValue(), Driver.LeftStickX.getValue(), -Driver.RightStickX.getValue(), State.MANUAL);
            } else {
                swerve.setAlignment(targetSnap.get());
            swerve.sendInput(-Driver.LeftStickY.getValue(), Driver.LeftStickX.getValue(), -Driver.RightStickX.getValue(), State.ALIGNMENT);
            }
            
        }
        else if(Driver.LeftBumper.isActive()){
            swerve.sendInput(-Driver.LeftStickY.getValue(), Driver.LeftStickX.getValue(), -Driver.RightStickX.getValue(), State.TARGETOBJECT);
        } else {
            swerve.sendInput(-Driver.LeftStickY.getValue(), Driver.LeftStickX.getValue(), -Driver.RightStickX.getValue(), State.MANUAL);
        }

    }

}
