// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.wcp.frc.Constants;

/** Add your docs here. */
public class TalonDefaultConfig {

    public static TalonFXConfiguration motionMagicConfig(){
        TalonFXConfiguration motionMagicConfig = new TalonFXConfiguration();

        motionMagicConfig.MotionMagic.MotionMagicAcceleration = ((int) (Constants.kSwerveRotationMaxSpeed * 15.5));
        motionMagicConfig.MotionMagic.MotionMagicCruiseVelocity = ((int) (Constants.kSwerveRotationMaxSpeed));

        // Slot 1 is for normal use
        motionMagicConfig.Slot0.kP =  1; // 1.55
        motionMagicConfig.Slot0.kI =  0.0;
        motionMagicConfig.Slot0.kD =  5.0; // 5.0
        motionMagicConfig.Slot0.kV = 1023.0 /Constants.kSwerveRotationMaxSpeed;
        
        return motionMagicConfig;
    }

    public static TalonFXConfiguration driveConfigs() {
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();



        driveConfigs.Slot0.kD=( 3.6);
        driveConfigs.Slot0.kV = 1023.0 / Constants.kSwerveDriveMaxSpeed;
        driveConfigs.Slot1.kP = 0;
        driveConfigs.Slot1.kI = 0.0;
        driveConfigs.Slot1.kD = 0;
        driveConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =.5; 

        driveConfigs.MotionMagic.MotionMagicCruiseVelocity = ((int) (Constants.kSwerveDriveMaxSpeed * 0.9));
        driveConfigs.MotionMagic.MotionMagicAcceleration = ((int) (Constants.kSwerveDriveMaxSpeed));

      return driveConfigs;
    }


}
