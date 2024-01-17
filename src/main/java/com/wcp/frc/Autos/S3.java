package com.wcp.frc.Autos;



import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.wcp.frc.subsystems.SuperStructure;
import com.wcp.frc.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class S3 extends AutoBase{
    SuperStructure s = SuperStructure.getInstance();
    Swerve swerve = Swerve.getInstance();

    PathPlannerTrajectory path = PathPlannerPath.fromPathFile("NS5").getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(180));

    @Override
    public void auto() {
        swerve.setTrajectory(path);
        swerve.startPath(false);
    }
}