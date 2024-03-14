package com.uni.frc.Autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.subsystems.Shooter;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class S4 extends AutoBase {
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive mSwerve = SwerveDrive.getInstance();
    double initRotation = -60;
    PathPlannerPath path = PathPlannerPath.fromPathFile("S4");
    
    PathPlannerTrajectory trajectory = path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation));

    @Override
    public void auto() {
        s.waitState(3, false);
        Shooter.getInstance().setPercent(0.8);
        // s.setPivotState(55);
        // s.shootState(false);
        s.trajectoryState(trajectory, initRotation);
        registerTrajectoryEvents("S4");

    }}