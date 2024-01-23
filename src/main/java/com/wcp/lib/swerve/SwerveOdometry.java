// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib.swerve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.ejml.simple.SimpleMatrix;

import com.wcp.frc.subsystems.Swerve.SwerveDriveModule;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class SwerveOdometry {
    private Pose2d m_poseMeters;
    private SwerveKinematics m_kinematics;
    private ChassisSpeeds m_velocity;
    private double m_previousTimestamp = -1;

    private Rotation2d m_previousAngle;
    private double[] m_PreviousDistances;

    public SwerveOdometry(SwerveKinematics kinematics,Pose2d initalPose, double... previousDistances){
        m_kinematics = kinematics;
        m_poseMeters = initalPose;
        m_velocity = new ChassisSpeeds();
        m_previousAngle = initalPose.getRotation();
        m_PreviousDistances = previousDistances;
    }

    public SwerveOdometry(SwerveKinematics kinematics, Pose2d initalPose){
        this(kinematics, initalPose, new double[4]);
    }

    public ChassisSpeeds getVelocity(){
        return m_velocity;
    }

    public Pose2d updateWithSwerveModuleStates(Rotation2d heading, List<SwerveDriveModule> modules, double timestamp){

            var positionModules = Arrays.asList(modules.get(0),modules.get(1),modules.get(2),modules.get(3));
            double x = 0.0;
            double y = 0.0;      
            double averageDistance = 0.0;
            double[] distances = new double[4];
            for (SwerveDriveModule m : positionModules) {
                m.updatePose(heading);
                double distance = m.getEstimatedRobotPose().getTranslation().translateBy(m_poseMeters.getTranslation().inverse())
                        .norm();
                distances[m.moduleID] = distance;
                averageDistance += distance;
            }
            averageDistance /= positionModules.size();

            m_velocity = m_kinematics.toChassisSpeedWheelConstraints(modules);

            int minDevianceIndex = 0;
            double minDeviance = Units.inchesToMeters(100);
            List<SwerveDriveModule> modulesToUse = new ArrayList<>();
            for (SwerveDriveModule m : positionModules) {
                double deviance = Math.abs(distances[m.moduleID] - averageDistance);
                if (deviance < minDeviance) {
                    minDeviance = deviance;
                    minDevianceIndex = m.moduleID;
                }
                if (deviance <= 10000) {
                    modulesToUse.add(m);
                }
            }
      
            if (modulesToUse.isEmpty()) {
                modulesToUse.add(modules.get(minDevianceIndex));
            }
            
            for (SwerveDriveModule m : modulesToUse) {
                x += m.getEstimatedRobotPose().getTranslation().x();
                y += m.getEstimatedRobotPose().getTranslation().y();
            }
            Pose2d updatedPose = new Pose2d(new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()), heading);
            Translation2d deltaPos = updatedPose.getTranslation().translateBy(m_poseMeters.getTranslation().inverse());
            m_velocity.vxMetersPerSecond = deltaPos.scale(1 / (timestamp - m_previousTimestamp)).x();
            m_velocity.vyMetersPerSecond = deltaPos.scale(1 / (timestamp - m_previousTimestamp)).y();
      
            m_poseMeters= updatedPose;
      
            modules.forEach((m) -> m.resetPose(m_poseMeters));
            m_previousTimestamp = timestamp;
            m_previousAngle = heading;

            return m_poseMeters;
    }

    public Pose2d getPoseMeters(){
        return m_poseMeters;
    }
    
    public void resetPosition(Pose2d pose, List<SwerveDriveModule> modules){
        modules.forEach((m) -> m.resetPose(pose));
    }
}
