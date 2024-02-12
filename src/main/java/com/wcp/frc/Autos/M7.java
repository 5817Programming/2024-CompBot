package com.wcp.frc.Autos;



import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.wcp.frc.subsystems.SuperStructure;
import com.wcp.frc.subsystems.Swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class M7 extends AutoBase{
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive swerve = SwerveDrive.getInstance();
    double initRotation = 0;
    PathPlannerTrajectory path = PathPlannerPath.fromPathFile("M7").getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation));

    @Override
    public void auto() {
        s.trajectoryState(path,9,initRotation);
        s.intakeState(false);
        s.waitState(15, false);
    }
}