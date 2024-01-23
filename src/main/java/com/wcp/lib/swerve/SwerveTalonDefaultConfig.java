// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.wcp.frc.Constants;

/** Add your docs here. */
public class SwerveTalonDefaultConfig {

    public static TalonFXConfiguration motionMagicConfig(){
        TalonFXConfiguration mSteerConfig = new TalonFXConfiguration();

        mSteerConfig.CurrentLimits.SupplyCurrentLimit = 120;
        mSteerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        mSteerConfig.CurrentLimits.StatorCurrentLimit = 120;
        mSteerConfig.CurrentLimits.StatorCurrentLimitEnable = false;

        mSteerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mSteerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        mSteerConfig.Slot0.kP = 6;
        mSteerConfig.Slot0.kS = 0.8;
        mSteerConfig.Slot0.kV = .1224;

        mSteerConfig.MotionMagic.MotionMagicCruiseVelocity = 98;
        mSteerConfig.MotionMagic.MotionMagicAcceleration = 1000;
        
        return mSteerConfig;
    }

    public static TalonFXConfiguration driveConfigs() {
        TalonFXConfiguration mDriveConfig = new TalonFXConfiguration();


        mDriveConfig.CurrentLimits.SupplyCurrentLimit = 120;
        mDriveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        mDriveConfig.CurrentLimits.StatorCurrentLimit = 120;
        mDriveConfig.CurrentLimits.StatorCurrentLimitEnable = false;

        mDriveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        mDriveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        mDriveConfig.Slot0.kI = 0.0;
        mDriveConfig.Slot0.kP = .24;
        mDriveConfig.Slot0.kD = 0.000002 * 12;
        mDriveConfig.Slot0.kV = 1 / 101.98 * 12;
        mDriveConfig.Slot0.kS = 0.8;
        // mDriveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =.5;

        mDriveConfig.MotionMagic.MotionMagicCruiseVelocity = ((int) (Constants.kSwerveDriveMaxSpeed * 0.9));
        mDriveConfig.MotionMagic.MotionMagicAcceleration = ((int) (Constants.kSwerveDriveMaxSpeed));

      return mDriveConfig;
    }


}
