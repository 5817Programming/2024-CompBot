package com.uni.frc.Autos;

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

        s.waitForPositionState(0.75);
        s.intakeState(.5);
        s.stopTrajectoryState();
        s.setPivotState(-.288);
        s.shootState(false);
        s.resumeTrajectoryState();

        s.waitForPositionState(2.2);
        s.intakeState(.5);
        s.stopTrajectoryState();
        s.setPivotState(-.299);
        s.shootState(false);
        s.resumeTrajectoryState();

        s.waitForPositionState(4.1);
        s.intakeState(.5);
        s.stopTrajectoryState();
        s.setPivotState(-.299);
        s.shootState(false);
        s.resumeTrajectoryState();

        s.waitForPositionState(5.3);
        s.intakeState(.7);

        s.waitForPositionState(7.5);
        s.stopTrajectoryState();
        s.setPivotState(-.3);
        s.shootState(false);
        s.resumeTrajectoryState();

    
        s.waitForPositionState(8.4);
        s.intakeState(.9);

        s.waitForPositionState(10.75);
        s.stopTrajectoryState();
        s.setPivotState(-.3);
        s.shootState(false);
        s.resumeTrajectoryState();
    

        // s.waitForPositionState(new Translation2d(1.98,5.1));
        // s.intakeState(.2);
        // s.waitForPositionState(new Translation2d(2.2,5.16));
        // s.shootState(false);

        // s.waitForPositionState(new Translation2d(2.03,5.54));
        // s.intakeState(.2);
        // s.waitForPositionState(new Translation2d(2.75,5.77));
        // s.shootState(false);

        // s.waitForPositionState(new Translation2d(2.15,6.6));
        // s.intakeState(.2);
        // s.waitForPositionState(new Translation2d(4.06,7.32));
        // s.shootState(false);

        // s.waitForPositionState(new Translation2d(7.21,7.45));
        // s.intakeState(false);
        // s.waitForPositionState(new Translation2d(5.59,6.19));
        // s.shootState(false);

        // s.waitForPositionState(new Translation2d(7.56,5.94));
        // s.intakeState(false);
        // s.waitForPositionState(new Translation2d(5.59,6.19));
        // s.shootState(false);

        // s.waitForPositionState(new Translation2d(7.81,4.81));
        // s.intakeState(false);
        // s.waitForPositionState(new Translation2d(5.05,5.1));
        // s.shootState(false);

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