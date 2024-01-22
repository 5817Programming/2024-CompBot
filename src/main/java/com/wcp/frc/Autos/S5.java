package com.wcp.frc.Autos;



import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.wcp.frc.subsystems.SuperStructure;
import com.wcp.frc.subsystems.Swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class S5 extends AutoBase{
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive swerve = SwerveDrive.getInstance();

    PathPlannerTrajectory path = PathPlannerPath.fromPathFile("NS5").getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(180));

    @Override
    public void auto() {
    }
}