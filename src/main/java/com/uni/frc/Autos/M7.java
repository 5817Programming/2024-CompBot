package com.uni.frc.Autos;



import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.subsystems.SuperStructure;
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
        s.trajectoryState(path,9,initRotation);
        s.shootState();

        s.waitForTrajectoryState(0.7);
        s.intakeState(false,1.8);
        s.waitForTrajectoryState(1.8);
        s.shootState();

        s.waitForTrajectoryState(2);
        s.intakeState(false,2.89);
        s.waitForTrajectoryState(2.89);
        s.shootState();

        s.waitForTrajectoryState(3.2);
        s.intakeState(false,3.9);
        s.waitForTrajectoryState(3.9);
        s.shootState();

        s.waitForTrajectoryState(5.2);
        s.intakeState(false,7.35);
        s.waitForTrajectoryState(7.35);
        s.shootState();

        s.waitForTrajectoryState(8.13);
        s.intakeState(false,10.15);
        s.waitForTrajectoryState(10.15);
        s.shootState();

        s.waitForTrajectoryState(11.24);
        s.intakeState(false,12.94);
        s.waitForTrajectoryState(12.94);
        s.shootState();



    }


}