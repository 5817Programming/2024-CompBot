// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.Autos;

import java.util.List;

import com.pathplanner.lib.path.EventMarker;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.Swerve.SwerveDrive;

/** 
 * This is an abstract class
 * abstract classes tell its child methods it needs to have or it will raise an errror
 * we have to implement a method called auto
 * this class will also give its child the stop auto method because all autos will need it
 */
public abstract class AutoBase {
    public abstract void auto();

    public void runAuto(){
        auto();

    }

    public void stopAuto(){
        SuperStructure.getInstance().clearQueues();
        SwerveDrive.getInstance().setState(SwerveDrive.State.MANUAL);
    }

}
