package com.uni.frc.Autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.Swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class M7 extends AutoBase {
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive mSwerve = SwerveDrive.getInstance();
    double initRotation = -2;
    PathPlannerPath path = PathPlannerPath.fromPathFile("M7 SPEED");
    PathPlannerTrajectory trajectory = path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation));

    @Override
    public void auto() {
        s.trajectoryState(trajectory, initRotation);
        s.shootState(false);

        s.waitForEventState(.6);
        s.intakeState(false);

        s.waitForEventState(1.5);
        s.shootState(false);

        s.waitForEventState(2.05);
        s.intakeState(false);

        s.waitForEventState(2.8);
        s.shootState(false);

        s.waitForEventState(3.36);
        s.intakeState(false);

        s.waitForEventState(4.23);
        s.shootState(false);

        s.waitForEventState(5.52);
        s.intakeState(false);

        s.waitForEventState(7.82);
        s.shootState(false);

        s.waitForEventState(8.6);
        s.intakeState(false);

        s.waitForEventState(10.92);
        s.shootState(false);

        s.waitForEventState(11.8);
        s.intakeState(false);

        s.waitForEventState(13.76);
        s.shootState(false);

    }

}