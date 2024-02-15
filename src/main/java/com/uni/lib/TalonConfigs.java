// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.lib;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.uni.frc.Constants;

/** Add your docs here. */
public class TalonConfigs {

    public static TalonFXConfiguration swerveRotationConfig(){
        TalonFXConfiguration motionMagicConfig = new TalonFXConfiguration();

        motionMagicConfig.Slot0.kP = 6;
        motionMagicConfig.Slot0.kS = 0.8;
        motionMagicConfig.Slot0.kV = .1224;
        motionMagicConfig.MotionMagic.MotionMagicCruiseVelocity = 98;
        motionMagicConfig.MotionMagic.MotionMagicAcceleration = 1000;
        
        return motionMagicConfig;
    }

    public static TalonFXConfiguration swerveDriveConfig() {
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveConfigs.Slot0.kV = .12;
        driveConfigs.Slot0.kS = .25;
        driveConfigs.Slot0.kA = .01;
        driveConfigs.Slot0.kP = .05;
        driveConfigs.Slot0.kI = 0;
        driveConfigs.Slot0.kD = 0;
        driveConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0; 

        driveConfigs.MotionMagic.MotionMagicCruiseVelocity = 80;
        driveConfigs.MotionMagic.MotionMagicAcceleration = 240;

      return driveConfigs;
    }

    public static TalonFXConfiguration pivotConfigs() {
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

    public static TalonFXConfiguration rackConfigs() {
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

    public static TalonFXConfiguration wristConfigs() {
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
    public static TalonFXConfiguration intakeConfigs() {
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveConfigs.Slot0.kV = .12;
        driveConfigs.Slot0.kS = .25;
        driveConfigs.Slot0.kA = .01;
        driveConfigs.Slot0.kP = .11;
        driveConfigs.Slot0.kI = 0;
        driveConfigs.Slot0.kD = 0;

        driveConfigs.MotionMagic.MotionMagicCruiseVelocity = ((int) (Constants.kSwerveDriveMaxSpeed * 0.9));
        driveConfigs.MotionMagic.MotionMagicAcceleration = ((int) (Constants.kSwerveDriveMaxSpeed));

      return driveConfigs;
    }
}
