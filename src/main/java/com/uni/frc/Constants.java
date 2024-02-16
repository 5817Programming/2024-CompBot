// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;
import java.util.Map;

import com.uni.lib.UndistortConstants;
import com.uni.lib.Vision.UndistortMap;
import com.uni.lib.Vision.UndistortMap_Limelight_A_640x480;
import com.uni.lib.Vision.UndistortMap_Limelight_B_640x480;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Translation2d;
import com.uni.lib.geometry.HeavilyInspired.Obstacle;
import com.uni.lib.motion.MotionProfileConstraints;
import com.uni.lib.util.InterpolatingDouble;
import com.uni.lib.util.InterpolatingTreeMap;
import com.uni.lib.util.InterpolatingUndisortMap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class Constants {
    public static final int kSlotIdx = 0;

    /**
     * Talon FX supports multiple (cascaded) PID loops. For
     * now we just want the primary one.
     */
    public static final int kPIDLoopIdx = 0;
    public static boolean isCompbot = hasMacAddress("00:80:2F:38:8F:8B");
    /**
     * set to zero to skip waiting for confirmation, set to nonzero to wait and
     * report to DS if action fails.
     */
    public static final int TIMEOUT_MILLISECONDS = 30;
    public static final int kCANTimeoutMs = 20;// The refresh rate of the periodic looper

    public static final double kRobotBaseWidth = 24.0; // The Robot Wheel Base Width
    public static final double kRobotBaseLength = 24.0;
    public static final double mRobotBaseWidth = Units.inchesToMeters(24); // The Robot Wheel Base Width
    public static final double mRobotBaseLength = Units.inchesToMeters(24);// The Robot Wheel Base Length

    public static final double kOuterWheelDriveDiameter = 4.0;

    // Kalman Filters
    public static final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(Math.pow(0.04, 1), Math.pow(0.04, 1));
    public static final Matrix<N2, N1> kLocalMeasurementStdDevs = VecBuilder.fill(Math.pow(0.01, 1), Math.pow(0.01, 1));
    public static final double kTagPoseRejectionFilter = 2.25; // Reject Vision Correction if Current Pose Farther than
                                                               // 3m from April Tag
    public static final double kNumInitialMeasurements = 15;// 100; //Number of Measurements to Let Into Filter Without
                                                            // Rejection to Initialize X_Hat

    // Sets the motor on a 2Dplane
    public static final Translation2d[] modulePositions = new Translation2d[] {
            new Translation2d(mRobotBaseWidth / 2, mRobotBaseLength / 2),
            new Translation2d(mRobotBaseWidth / 2, -mRobotBaseLength / 2),
            new Translation2d(-mRobotBaseWidth / 2, mRobotBaseLength / 2),
            new Translation2d(-mRobotBaseWidth / 2, -mRobotBaseLength / 2),

    };
    // The positions of the modules, relative to the robot's center
    public static final Translation2d kFrontRightPosition = new Translation2d(kRobotBaseWidth / 2,
            kRobotBaseLength / 2);
    public static final Translation2d kFrontLeftPosition = new Translation2d(kRobotBaseWidth / 2,
            -kRobotBaseLength / 2);
    public static final Translation2d kRearLeftPosition = new Translation2d(-kRobotBaseWidth / 2,
            -kRobotBaseLength / 2);
    public static final Translation2d kRearRightPosition = new Translation2d(-kRobotBaseWidth / 2,
            kRobotBaseLength / 2);
    public static final Translation2d mFrontRightPosition = new Translation2d(mRobotBaseWidth / 2,
            mRobotBaseLength / 2);
    public static final Translation2d mFrontLeftPosition = new Translation2d(mRobotBaseWidth / 2,
            -mRobotBaseLength / 2);
    public static final Translation2d mRearLeftPosition = new Translation2d(-mRobotBaseWidth / 2,
            -mRobotBaseLength / 2);
    public static final Translation2d mRearRightPosition = new Translation2d(-mRobotBaseWidth / 2,
            mRobotBaseLength / 2);

    public static final List<Translation2d> kModulePositions = Arrays.asList(kFrontRightPosition, kFrontLeftPosition,
            kRearLeftPosition, kRearRightPosition);

    public static double kSwerveRotationMaxSpeed = 12720 * 0.8;
    public static double GrannyModeWeight = .5;

    // Scrub Factors
    public static final boolean kSimulateReversedCarpet = false;
    public static final double[] kWheelScrubFactors = new double[] { 1.0, 1.0, 1.0, 1.0 };
    public static final double kXScrubFactorP = isCompbot?0.935:0.85;
    public static final double kYScrubFactorP = isCompbot?1:.875;
    public static final double kXScrubFactorN = isCompbot?0.935:0.85;
    public static final double kYScrubFactorN = isCompbot?.86:.875;


    public static final double driveKS = (0.32 / 12);
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    public static final double kSwerveMaxspeedMPS = 10;
    public static final double SwerveMaxspeedMPS = 120;
    public static final double kSwerveDriveMaxSpeed = 22000.0; // The theoretical max speed(For the Falcon 500s)
    public static final double kSwerveRotation10VoltMaxSpeed = 1350.0;

    public static final double kSwerveRotationReduction = Options.rotationRatio; // The Module to Motor Ratio(i.e,
                                                                                 // amount the rotation motor rotates,
                                                                                 // for every one rotation for the
                                                                                 // module)
    public static final double kSwerveWheelReduction = Options.driveRatio; // The Wheel to Motor Ratio(i.e, amount the
                                                                           // drive motor rotates, for every one
                                                                           // rotation for the wheel)
    public static final double kSwerveRotationEncoderResolution = 2048.0;
    public static final double kSwerveDriveEncoderResolution = 2048.0;

    public static final double kSwerveWheelDiameter = 4; // inches

    public static final double kSwerveEncUnitsPerWheelRev = kSwerveDriveEncoderResolution * 2048;
    public static final double kSwerveEncUnitsPerInch = kSwerveEncUnitsPerWheelRev / (Math.PI * kSwerveWheelDiameter);
    public static final double kWheelCircumference = Units.inchesToMeters(kSwerveWheelDiameter * Math.PI);

    public static final double kMaxVelocityMetersPerSecond = 5.05; // Calibrated 3/12 on Comp Bot
    public static final double kMaxAccelerationMetersPerSecondSquared = 4.4;

    public static final MotionProfileConstraints kPositionMotionProfileConstraints = new MotionProfileConstraints(
            0.8 * Constants.kMaxVelocityMetersPerSecond,
            0.8 * -Constants.kMaxVelocityMetersPerSecond,
            0.6 * Constants.kMaxAccelerationMetersPerSecondSquared);
    public static final MotionProfileConstraints kHeadingMotionProfileConstraints = new MotionProfileConstraints(
            0.5 * Constants.kMaxAngularSpeedRadiansPerSecond,
            0.5 * -Constants.kMaxAngularSpeedRadiansPerSecond,
            1.0 * Constants.kMaxAngularAccelerationRadiansPerSecondSquared);
    /// The absolute starting postion for each module
    public static final double kMaxAngularSpeedRadiansPerSecond = kMaxVelocityMetersPerSecond /
    Math.hypot(mRobotBaseLength / 2.0, mRobotBaseWidth / 2.0);
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = kMaxAccelerationMetersPerSecondSquared /
    Math.hypot(mRobotBaseLength / 2.0, mRobotBaseWidth / 2.0);

    public static final double kFrontRightStartingEncoderPosition = isCompbot ? -103.3 : -86; // -354.950352
    public static final double kFrontLeftStartingEncoderPosition = isCompbot ? 52.25 : -160; // -263.094811
    public static final double kRearLeftStartingEncoderPosition = isCompbot ? -138.2: -355; // -121.094031
    public static final double kRearRightStartingEncoderPosition =isCompbot ? 10.2 : -265; // -355.170825

    public static final class ShooterConstants {
        public static final double HANDOFF = 0;
        public static final double IDLE = 0;
        public static final double FEEDFORWARD = 2;

        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> SHOT_TRAVEL_TIME_TREE_MAP = new InterpolatingTreeMap<>();
        static {
            SHOT_TRAVEL_TIME_TREE_MAP.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
            SHOT_TRAVEL_TIME_TREE_MAP.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
            SHOT_TRAVEL_TIME_TREE_MAP.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
            SHOT_TRAVEL_TIME_TREE_MAP.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
        }
    }

    public static final class IntakeConstants {
        public static final double RAMP = 0;
        public static final double HANDOFF = 0;
        public static final double IDLE = 0;
    }

    public static final class IndexerConstants {
        public static final double RAMP = 0;
        public static final double HANDOFF = 0;
        public static final double IDLE = 0;
    }

    public static final class ElevatorConstants {
        public static final double SPEAKER = 0;
        public static final double AMP = 0;
        public static final double TRAP = 0;
        public static final double TRANSFER = 0;
        public static final double SHOOTING = 0;
        public static final double MAX_UP = 0;
        public static final double MAX_DOWN = 0;
    }

    public static final class WristConstants {
        public static final double AMP = 0;
        public static final double TRAP = 0;
        public static final double TRANSFER = 0;
        public static final double SHOOTING = 0;
        public static final double MAX_UP = 0;
        public static final double MAX_DOWN = 0;
    }

    public static final class PivotConstants {
        public static final double SPEAKER = 0;
        public static final double AMP = 0;
        public static final double TRAP = 0;
        public static final double TRANSFER = 0;
        public static final double SHOOTING = 0;
        public static final double MAX_UP = 0;
        public static final double MAX_DOWN = 0;

        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> ANGLE_TREEMAP = new InterpolatingTreeMap<>();
        static {
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
        public static final int APRIL_PIPLINE = 0;
        public static final int LIMELIGHT_LENS_HEIGHT_INCHES = 25;

        public static final Pose2d ROBOT_TO_CAMERA = new Pose2d(new Translation2d(.072,-.07),
                Rotation2d.fromDegrees(0));

        public static final Rotation2d cameraYawOffset = Rotation2d.fromDegrees(0);
        public static final Rotation2d HORIZONTAL_PLANE_TO_LENSE = Rotation2d.fromDegrees(21);

        public static final double IMAGE_CAPTURE_LATENCY = 11.0;
       

        public static final UndistortConstants UNDISTORT_CONSTANTS =  new UndistortConstants(
                new double[]{0.15545342, -0.46477633, 0.00277053, 0.0030637, 0.39464241},
                new double[][]{{0.79862571, 0., 0.46119489},
                        {0., 1.06276288, 0.48098825},
                        {0., 0., 1.}}
                );

        public static final UndistortMap UNDISTORTMAP = new InterpolatingUndisortMap((int)1280.0, (int)960.0, new UndistortMap_Limelight_B_640x480());
    }

    public static final class LightConstants {
        public final static double NORMAL_LIGHT = -0.99;// rainbow light

        public final static double CUBE_LIGHT = 0.57;// purple light

        public final static double CONE_LIGHT = 0.65;// yellow light
    }
    
public static final class FieldConstants {
        public static final double fieldLength = Units.inchesToMeters(651.25);
        public static final double fieldWidth = Units.inchesToMeters(315.5);
        public static final double tapeWidth = Units.inchesToMeters(2.0);
    
        // Dimensions for community and charging station, including the tape.
        public static final class Community {
            // Region dimensions
            public static final double innerX = 0.0;
            public static final double midX = Units.inchesToMeters(132.375); // Tape to the left of charging station
            public static final double outerX = Units.inchesToMeters(193.25); // Tape to the right of charging station
            public static final double leftY = Units.feetToMeters(18.0);
            public static final double midY = leftY - Units.inchesToMeters(59.39) + tapeWidth;
            public static final double rightY = 0.0;
            public static final Translation2d[] regionCorners = new Translation2d[] {
                    new Translation2d(innerX, rightY),
                    new Translation2d(innerX, leftY),
                    new Translation2d(midX, leftY),
                    new Translation2d(midX, midY),
                    new Translation2d(outerX, midY),
                    new Translation2d(outerX, rightY),
            };
    
            // Charging station dimensions
            public static final double chargingStationLength = Units.inchesToMeters(76.125);
            public static final double chargingStationWidth = Units.inchesToMeters(97.25);
            public static final double chargingStationOuterX = outerX - tapeWidth;
            public static final double chargingStationInnerX = chargingStationOuterX - chargingStationLength;
            public static final double chargingStationLeftY = midY - tapeWidth;
            public static final double chargingStationRightY = chargingStationLeftY - chargingStationWidth;
            public static final Translation2d[] chargingStationCorners = new Translation2d[] {
                    new Translation2d(chargingStationInnerX, chargingStationRightY),
                    new Translation2d(chargingStationInnerX, chargingStationLeftY),
                    new Translation2d(chargingStationOuterX, chargingStationRightY),
                    new Translation2d(chargingStationOuterX, chargingStationLeftY)
            };
            public static final Translation2d[] wallCorners = new Translation2d[] {
                new Translation2d(3.5, 5.34),
                new Translation2d(3.5, 5.70),
                new Translation2d(-100, 5.34),
                new Translation2d(-100, 5.70)
        }; 
    
            // Cable bump
            public static final double cableBumpInnerX = innerX + Grids.outerX + Units.inchesToMeters(95.25);
            public static final double cableBumpOuterX = cableBumpInnerX + Units.inchesToMeters(7);
            public static final Translation2d[] cableBumpCorners = new Translation2d[] {
                    new Translation2d(cableBumpInnerX, 0.0),
                    new Translation2d(cableBumpInnerX, chargingStationRightY),
                    new Translation2d(cableBumpOuterX, 0.0),
                    new Translation2d(cableBumpOuterX, chargingStationRightY)
            };
        }
    
        // Dimensions for grids and nodes
        public static final class Grids {
            // X layout
            public static final double outerX = Units.inchesToMeters(54.25);
            public static final double lowX = outerX - (Units.inchesToMeters(14.25) / 2.0); // Centered when under cube
                                                                                            // nodes
            public static final double midX = outerX - Units.inchesToMeters(22.75);
            public static final double highX = outerX - Units.inchesToMeters(39.75);
    
            // Y layout
            public static final int nodeRowCount = 9;
            public static final double nodeFirstY = Units.inchesToMeters(20.19);
            public static final double nodeSeparationY = Units.inchesToMeters(22.0);
    
            // Z layout
            public static final double cubeEdgeHigh = Units.inchesToMeters(3.0);
            public static final double highCubeZ = Units.inchesToMeters(35.5) - cubeEdgeHigh;
            public static final double midCubeZ = Units.inchesToMeters(23.5) - cubeEdgeHigh;
            public static final double highConeZ = Units.inchesToMeters(46.0);
            public static final double midConeZ = Units.inchesToMeters(34.0);
    
            // Translations (all nodes in the same column/row have the same X/Y coordinate)
            public static final Translation2d[] lowTranslations = new Translation2d[nodeRowCount];
            public static final Translation2d[] midTranslations = new Translation2d[nodeRowCount];
            public static final Translation3d[] mid3dTranslations = new Translation3d[nodeRowCount];
            public static final Translation2d[] highTranslations = new Translation2d[nodeRowCount];
            public static final Translation3d[] high3dTranslations = new Translation3d[nodeRowCount];
    
            static {
                for (int i = 0; i < nodeRowCount; i++) {
                    boolean isCube = i == 1 || i == 4 || i == 7;
                    lowTranslations[i] = new Translation2d(lowX, nodeFirstY + nodeSeparationY * i);
                    midTranslations[i] = new Translation2d(midX, nodeFirstY + nodeSeparationY * i);
                    mid3dTranslations[i] = new Translation3d(midX, nodeFirstY + nodeSeparationY * i,
                            isCube ? midCubeZ : midConeZ);
                    high3dTranslations[i] = new Translation3d(
                            highX, nodeFirstY + nodeSeparationY * i, isCube ? highCubeZ : highConeZ);
                    highTranslations[i] = new Translation2d(highX, nodeFirstY + nodeSeparationY * i);
                }
            }
    
            // Complex low layout (shifted to account for cube vs cone rows and wide edge
            // nodes)
            public static final double complexLowXCones = outerX - Units.inchesToMeters(16.0) / 2.0; // Centered X under
                                                                                                     // cone nodes
            public static final double complexLowXCubes = lowX; // Centered X under cube nodes
            public static final double complexLowOuterYOffset = nodeFirstY - Units.inchesToMeters(3.0)
                    - (Units.inchesToMeters(25.75) / 2.0);
    
            public static final Translation2d[] complexLowTranslations = new Translation2d[] {
                    new Translation2d(complexLowXCones, nodeFirstY - complexLowOuterYOffset),
                    new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 1),
                    new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 2),
                    new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 3),
                    new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 4),
                    new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 5),
                    new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 6),
                    new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 7),
                    new Translation2d(
                            complexLowXCones, nodeFirstY + nodeSeparationY * 8 + complexLowOuterYOffset),
            };
        }
    
        // Dimensions for loading zone and substations, including the tape
        public static final class LoadingZone {
            // Region dimensions
            public static final double width = Units.inchesToMeters(99.0);
            public static final double innerX = FieldConstants.fieldLength;
            public static final double midX = fieldLength - Units.inchesToMeters(132.25);
            public static final double outerX = fieldLength - Units.inchesToMeters(264.25);
            public static final double leftY = FieldConstants.fieldWidth;
            public static final double midY = leftY - Units.inchesToMeters(50.5);
            public static final double rightY = leftY - width;
            public static final Translation2d[] regionCorners = new Translation2d[] {
                    new Translation2d(
                            midX, rightY), // Start at lower left next to border with opponent community
                    new Translation2d(midX, midY),
                    new Translation2d(outerX, midY),
                    new Translation2d(outerX, leftY),
                    new Translation2d(innerX, leftY),
                    new Translation2d(innerX, rightY),
            };
    
            // Double substation dimensions
            public static final double doubleSubstationLength = Units.inchesToMeters(14.0);
            public static final double doubleSubstationX = innerX - doubleSubstationLength;
            public static final double doubleSubstationShelfZ = Units.inchesToMeters(37.375);
    
            // Single substation dimensions
            public static final double singleSubstationWidth = Units.inchesToMeters(22.75);
            public static final double singleSubstationLeftX = FieldConstants.fieldLength - doubleSubstationLength
                    - Units.inchesToMeters(88.77);
            public static final double singleSubstationCenterX = singleSubstationLeftX + (singleSubstationWidth / 2.0);
            public static final double singleSubstationRightX = singleSubstationLeftX + singleSubstationWidth;
            public static final Translation2d singleSubstationTranslation = new Translation2d(singleSubstationCenterX,
                    leftY);
    
            public static final double singleSubstationHeight = Units.inchesToMeters(18.0);
            public static final double singleSubstationLowZ = Units.inchesToMeters(27.125);
            public static final double singleSubstationCenterZ = singleSubstationLowZ + (singleSubstationHeight / 2.0);
            public static final double singleSubstationHighZ = singleSubstationLowZ + singleSubstationHeight;
        }
    
        // Locations of staged game pieces
        public static final class StagingLocations {
            public static final double centerOffsetX = Units.inchesToMeters(47.36);
            public static final double positionX = fieldLength / 2.0 - Units.inchesToMeters(47.36);
            public static final double firstY = Units.inchesToMeters(36.19);
            public static final double separationY = Units.inchesToMeters(48.0);
            public static final Translation2d[] translations = new Translation2d[4];
    
            static {
                for (int i = 0; i < translations.length; i++) {
                    translations[i] = new Translation2d(positionX, firstY + (i * separationY));
                }
            }
        }
    
        // AprilTag locations (do not flip for red alliance)
        public static final Map<Integer, Pose3d> aprilTags = Map.of(
                1,
                new Pose3d(
                        Units.inchesToMeters(610.77),
                        Units.inchesToMeters(42.19),
                        Units.inchesToMeters(18.22),
                        new Rotation3d(0.0, 0.0, Math.PI)),
                2,
                new Pose3d(
                        Units.inchesToMeters(610.77),
                        Units.inchesToMeters(108.19),
                        Units.inchesToMeters(18.22),
                        new Rotation3d(0.0, 0.0, Math.PI)),
                3,
                new Pose3d(
                        Units.inchesToMeters(610.77),
                        Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                        Units.inchesToMeters(18.22),
                        new Rotation3d(0.0, 0.0, Math.PI)),
                4,
                new Pose3d(
                        Units.inchesToMeters(636.96),
                        Units.inchesToMeters(265.74),
                        Units.inchesToMeters(27.38),
                        new Rotation3d(0.0, 0.0, Math.PI)),
                5,
                new Pose3d(
                        Units.inchesToMeters(14.25),
                        Units.inchesToMeters(265.74),
                        Units.inchesToMeters(27.38),
                        new Rotation3d()),
                6,
                new Pose3d(
                        Units.inchesToMeters(40.45),
                        Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                        Units.inchesToMeters(18.22),
                        new Rotation3d()),
                7,
                new Pose3d(
                        Units.inchesToMeters(40.45),
                        Units.inchesToMeters(108.19),
                        Units.inchesToMeters(18.22),
                        new Rotation3d()),
                8,
                new Pose3d(
                        Units.inchesToMeters(40.45),
                        Units.inchesToMeters(42.19),
                        Units.inchesToMeters(18.22),
                        new Rotation3d()));
    
        
        
    
        public static List<Obstacle> obstacles = List.of(
                // Blue Charging Station
              
                );
            }
        public static Translation2d allianceFlip(Translation2d translation) {
            
            if (true) {
                return new Translation2d(Units.inchesToMeters(651.25) - translation.x(), translation.y());
            } else {
                return translation;
            }
            
        }
 /**
     * Check if this system has a certain mac address in any network device.
     * @param mac_address Mac address to check.
     * @return true if some device with this mac address exists on this system.
     */
    public static boolean hasMacAddress(final String mac_address) {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis == null) {
                    continue;
                }
                StringBuilder device_mac_sb = new StringBuilder();
                System.out.println("hasMacAddress: NIS: " + nis.getDisplayName());
                byte[] mac = nis.getHardwareAddress();
                if (mac != null) {
                    for (int i = 0; i < mac.length; i++) {
                        device_mac_sb.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
                    }
                    String device_mac = device_mac_sb.toString();
                    System.out.println("hasMacAddress: NIS " + nis.getDisplayName() + " device_mac: " + device_mac);
                    if (mac_address.equals(device_mac)) {
                        System.out.println("hasMacAddress: ** Mac address match! " + device_mac);
                        return true;
                    }
                } else {
                    System.out.println("hasMacAddress: Address doesn't exist or is not accessible");
                }
            }

        } catch (SocketException e) {
            e.printStackTrace();
        }
        return false;
    }
}
