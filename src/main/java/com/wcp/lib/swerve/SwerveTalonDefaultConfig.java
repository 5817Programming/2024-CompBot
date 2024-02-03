// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.wcp.frc.Constants;

/** Add your docs here. */
public class SwerveTalonDefaultConfig {

    public static TalonFXConfiguration motionMagicConfig(){
        TalonFXConfiguration motionMagicConfig = new TalonFXConfiguration();

 motionMagicConfig.Slot0.kP = 6;
        motionMagicConfig.Slot0.kS = 0.8;
        motionMagicConfig.Slot0.kV = .1224;



        motionMagicConfig.MotionMagic.MotionMagicCruiseVelocity = 98;
        motionMagicConfig.MotionMagic.MotionMagicAcceleration = 1000;
        
        return motionMagicConfig;
    }

    public static TalonFXConfiguration driveConfigs() {
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();



        driveConfigs.Slot0.kV = .12;
        driveConfigs.Slot0.kS = .25;
        driveConfigs.Slot0.kA = .01;
        driveConfigs.Slot0.kP = .11;
        driveConfigs.Slot0.kI = 0;
        driveConfigs.Slot0.kD = 0;
        driveConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =.5; 

        driveConfigs.MotionMagic.MotionMagicCruiseVelocity = ((int) (Constants.kSwerveDriveMaxSpeed * 0.9));
        driveConfigs.MotionMagic.MotionMagicAcceleration = ((int) (Constants.kSwerveDriveMaxSpeed));

      return driveConfigs;
    }


}
