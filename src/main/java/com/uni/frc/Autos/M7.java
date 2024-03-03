package com.uni.frc.Autos;

import java.util.Arrays;

import com.fasterxml.jackson.core.util.DefaultPrettyPrinter.Indenter;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.subsystems.Indexer;
import com.uni.frc.subsystems.Intake;
import com.uni.frc.subsystems.Shooter;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Translation2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class M7 extends AutoBase {
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive mSwerve = SwerveDrive.getInstance();
    double initRotation = 0;
    PathPlannerPath path = PathPlannerPath.fromPathFile("M7 SPEED");
    PathPlannerTrajectory trajectory = path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation));

    @Override
    public void auto() {
        Shooter.getInstance().setPercent(1);
        //Shot 1
        s.setPivotState(-.266);
        s.shootState(false);
        s.trajectoryState(trajectory, initRotation);
        registerTrajectoryStops(Arrays.asList(1.13,2.93,4.28,7.51,10.67));

        s.waitForPositionState(0.75);
        s.intakeState(.5,-.288);
        s.stopTrajectoryState();
        s.shootState(false);
        s.resumeTrajectoryState();

        s.waitForPositionState(2.2);
        s.intakeState(.5,-.299);
        s.shootState(false);
        s.resumeTrajectoryState();

        s.waitForPositionState(4.1);
        s.intakeState(.5,-.299);
        s.shootState(false);
        s.resumeTrajectoryState();

        s.waitForPositionState(5.3);
        s.intakeState(.7);

        s.waitForPositionState(7.5);
        s.setPivotState(-.3);
        s.shootState(false);
        s.resumeTrajectoryState();

    
        s.waitForPositionState(8.4);
        s.intakeState(.9);

        s.waitForPositionState(10.75);
        s.setPivotState(-.3);
        s.shootState(false);
        s.resumeTrajectoryState();
    }
    @Override
    public void testAuto() {
        // s.shootState(false);
        s.trajectoryState(trajectory, initRotation);

        // s.waitForPositionState(.5);
        s.intakeState(1);
        // s.shootState(false);

        // s.waitForPositionState(2);
        // s.intakeState(1);
        // s.shootState(false);

        // s.waitForPositionState(3.5);
        // s.intakeState(1);
        // s.shootState(false);

        // s.waitForPositionState(5.73);
        // s.intakeState(false);
        // s.waitForPositionState(7.86);
        // s.shootState(false);

        // s.waitForPositionState(8.8);
        // s.intakeState(false);
        // s.waitForPositionState(11.1);
        // s.shootState(false);

        // s.waitForPositionState(12.2);
        // s.intakeState(false);
        // s.waitForPositionState(14.25);
        // s.shootState(false);

    }
}