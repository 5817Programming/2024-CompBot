package com.uni.frc.Autos;



import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.Swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class M5 extends AutoBase{
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive swerve = SwerveDrive.getInstance();
    double initRotation = 0;
    PathPlannerTrajectory path = PathPlannerPath.fromPathFile("NS5").getTrajectory(new ChassisSpeeds(),  Rotation2d.fromDegrees(initRotation));

    @Override
    public void auto() {
        s.trajectoryState(path,5,initRotation);
        s.waitState(15, false);
    }
}