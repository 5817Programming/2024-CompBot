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
import com.uni.lib.motion.PathStateGenerator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class M7 extends AutoBase {
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive mSwerve = SwerveDrive.getInstance();
    double initRotation = 0;
    PathPlannerPath path = PathPlannerPath.fromPathFile("M6 SPEED");
    PathPlannerPath testPath = PathPlannerPath.fromPathFile("M7 SPEED");
    PathPlannerTrajectory trajectory = path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation));
    PathPlannerTrajectory testTrajectory = path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation));

    @Override
    public void auto() {
        Shooter.getInstance().setPercent(1);
        //Shot 1
        PathStateGenerator.getInstance().setTrajectory(trajectory);
        s.setPivotState(-.179);
        s.shootState(false);
        s.trajectoryState(trajectory, initRotation);
        registerTrajectoryStops(Arrays.asList(1.13,2.93,4.2,7.51,10.67));

        s.waitForPositionState(0.6);
        s.intakeState(.7,-.288);
        s.stopTrajectoryState();
        s.shootState(false);
        s.resumeTrajectoryState();

        s.waitForPositionState(2);
        s.intakeState(.7,-.278);
        s.shootState(false);
        s.resumeTrajectoryState();

        s.waitForPositionState(3.9);
        s.intakeState(.7,-.284);
        s.shootState(false);
        s.resumeTrajectoryState();

        s.waitForPositionState(5);

        s.intakeState(1);
        
        s.waitForPositionState(6);
        s.setPivotState(-.34);

        s.waitForPositionState(7);
        s.shootState(false);
        s.resumeTrajectoryState();

    
        s.waitForPositionState(7);
        s.intakeState(2.7);
        s.waitForPositionState(10);

        s.setPivotState(-.34);

        s.waitForPositionState(10.75);
        s.shootState(false);
        s.resumeTrajectoryState();
    }
    @Override
    public void testAuto() {
        Shooter.getInstance().setPercent(1);
        Intake.getInstance().intakePercent(-1);
        Indexer.getInstance().setPercent(-1);
        s.trajectoryState(testTrajectory, initRotation);

    }
}