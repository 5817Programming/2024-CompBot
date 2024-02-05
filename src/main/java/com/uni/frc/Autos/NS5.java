package com.uni.frc.Autos;



import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.Swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class NS5 extends AutoBase{
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive swerve = SwerveDrive.getInstance();
    double initRotation = 180-62;
    PathPlannerTrajectory path = PathPlannerPath.fromPathFile("NS5").getTrajectory(new ChassisSpeeds(),  Rotation2d.fromDegrees(180));

    @Override
    public void auto() {
        s.trajectoryState(path,6,initRotation);
        s.waitState(15, false);
    }
}