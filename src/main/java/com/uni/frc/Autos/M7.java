package com.uni.frc.Autos;



import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.Requests.Prerequisite;
import com.uni.frc.subsystems.Swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class M7 extends AutoBase{
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive mSwerve = SwerveDrive.getInstance();
    double initRotation = -2;
    PathPlannerTrajectory path = PathPlannerPath.fromPathFile("M7 SPEED").getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation));

    @Override
    public void auto() {
        // s.intakeState(false);
        s.trajectoryState(path,9,initRotation);
        s.intakeState(false);
        s.waitState(25, false);
    }


}