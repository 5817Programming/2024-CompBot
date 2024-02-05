// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc;

import java.util.Arrays;
import java.util.List;

import com.uni.lib.geometry.Translation2d;
import com.uni.lib.util.InterpolatingDouble;
import com.uni.lib.util.InterpolatingTreeMap;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class CompConstants {
    public static final int kSlotIdx = 0;

	/**
	 * Talon FX supports multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int TIMEOUT_MILLISECONDS = 30;
    public static final int kCANTimeoutMs = 20;//The refresh rate of the periodic looper


    public static final double kRobotBaseWidth = 22.0; //The Robot Wheel Base Width
    public static final double kRobotBaseLength = 22.0; 
    public static final double mRobotBaseWidth = Units.inchesToMeters(22); //The Robot Wheel Base Width
    public static final double mRobotBaseLength = Units.inchesToMeters(22);//The Robot Wheel Base Length

    public static final double kOuterWheelDriveDiameter = 4.0;

    //Sets the motor on a 2Dplane
    public static final Translation2d[] modulePositions = new Translation2d[] {
        new Translation2d(mRobotBaseWidth / 2, mRobotBaseLength / 2),
        new Translation2d(mRobotBaseWidth / 2, -mRobotBaseLength / 2),
        new Translation2d(-mRobotBaseWidth / 2, mRobotBaseLength / 2),
        new Translation2d(-mRobotBaseWidth / 2, -mRobotBaseLength / 2),
      
    };
    //The positions of the modules, relative to the robot's center
    public static final Translation2d kFrontRightPosition = new Translation2d(kRobotBaseWidth / 2, kRobotBaseLength / 2);
    public static final Translation2d kFrontLeftPosition = new Translation2d(kRobotBaseWidth / 2, -kRobotBaseLength / 2);
    public static final Translation2d kRearLeftPosition = new Translation2d(-kRobotBaseWidth / 2, -kRobotBaseLength / 2);
    public static final Translation2d kRearRightPosition = new Translation2d(-kRobotBaseWidth / 2, kRobotBaseLength / 2);
    public static final Translation2d mFrontRightPosition = new Translation2d(mRobotBaseWidth / 2, mRobotBaseLength / 2);
    public static final Translation2d mFrontLeftPosition = new Translation2d(mRobotBaseWidth / 2, -mRobotBaseLength / 2);
    public static final Translation2d mRearLeftPosition = new Translation2d(-mRobotBaseWidth / 2, -mRobotBaseLength / 2);
    public static final Translation2d mRearRightPosition = new Translation2d(-mRobotBaseWidth / 2, mRobotBaseLength / 2);
    
    public static final List<Translation2d> kModulePositions = Arrays.asList(kFrontRightPosition, kFrontLeftPosition, kRearLeftPosition, kRearRightPosition); 


    public static double kSwerveRotationMaxSpeed = 12720 * 0.8;
    public static double GrannyModeWeight = .5;

    	//Scrub Factors
	public static final boolean kSimulateReversedCarpet = false;
	public static final double[] kWheelScrubFactors = new double[]{1.0, 1.0, 1.0, 1.0};
	public static final double kXScrubFactor =0.877;
	public static final double kYScrubFactor =0.845;
    
    public static final double driveKS = (0.32 / 12);
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    public static final double kSwerveMaxspeedMPS = 10;
    public static final double SwerveMaxspeedMPS = 4.5;
    public static final double kSwerveDriveMaxSpeed = 22000.0; //The theoretical max speed(For the Falcon 500s)
    public static final double kSwerveRotation10VoltMaxSpeed = 1350.0;

    public static final double kSwerveRotationReduction = Options.rotationRatio; //The Module to Motor Ratio(i.e, amount the rotation motor rotates, for every one rotation for the module)
    public static final double kSwerveWheelReduction = Options.driveRatio; //The Wheel to Motor Ratio(i.e, amount the drive motor rotates, for every one rotation for the wheel)
    public static final double kSwerveRotationEncoderResolution = 2048.0;
    public static final double kSwerveDriveEncoderResolution = 2048.0; 

    public static final double kSwerveWheelDiameter = 4; //inches 

    public static final double kSwerveEncUnitsPerWheelRev = kSwerveDriveEncoderResolution * 2048;
	public static final double kSwerveEncUnitsPerInch = kSwerveEncUnitsPerWheelRev / (Math.PI * kSwerveWheelDiameter);
    public static final double kWheelCircumference = Units.inchesToMeters(kSwerveWheelDiameter*Math.PI);


    ///The absolute starting postion for each module
    //originally +180 to each
    public static final double kFrontRightStartingEncoderPosition = -81.6; //-354.950352
    public static final double kFrontLeftStartingEncoderPosition = -287.3; //-263.094811
    public static final double kRearLeftStartingEncoderPosition = -76.2; //-121.094031
    public static final double kRearRightStartingEncoderPosition = -112.1; //-355.170825    
        
        
    public static final class ShooterConstants{
        public static final double HANDOFF = 0;
        public static final double IDLE = 0;
    }
    
    public static final class ElevatorConstants{
        public static final double SPEAKER = 0;
        public static final double AMP = 0;
        public static final double TRAP = 0;
        public static final double TRANSFER = 0;
        public static final double SHOOTING = 0;
        public static final double MAX_UP = 0;
        public static final double MAX_DOWN = 0;
    }

    public static final class WristConstants{
        public static final double AMP = 0;
        public static final double TRAP = 0;
        public static final double TRANSFER = 0;
        public static final double SHOOTING = 0;
        public static final double MAX_UP = 0;
        public static final double MAX_DOWN = 0;
    }

    public static final class PivotConstants{
        public static final double SPEAKER = 0;
        public static final double AMP = 0;
        public static final double TRAP = 0;
        public static final double TRANSFER = 0;
        public static final double SHOOTING = 0;
        public static final double MAX_UP = 0;
        public static final double MAX_DOWN = 0;

        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> ANGLE_TREEMAP = new InterpolatingTreeMap<>();
        static{
            ANGLE_TREEMAP.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
            ANGLE_TREEMAP.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
            ANGLE_TREEMAP.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
            ANGLE_TREEMAP.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
        }
    }

    public static final class ArmConstants {
        public static final double SPEAKER = 0;
        public static final double AMP = 0;
        public static final double TRAP = 0;
        public static final double TRANSFER = 0;
        public static final double SHOOTING = 0;
        public static final double MAX_UP = 0;
        public static final double MAX_DOWN = 0;
    }

    public static final class VisionConstants {
        public static final double MAX_UP = 0;
        public static final double MAX_DOWN = 0;
        public static final int APRIL_PIPLINE = 0;
        public static final int APRIL_HEIGHT_INCHES = 0;
        public static final int LIMELIGHT_LENS_HEIGHT_INCHES = 0;
        public static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 0;
    }

    public static final class LightConstants {
        public final static double NORMAL_LIGHT = -0.99;//rainbow light

        public final static double CUBE_LIGHT = 0.57;//purple light

        public final static double CONE_LIGHT = 0.65;//yellow light
    }

    public static final class FieldConstants {
    }
    
}

