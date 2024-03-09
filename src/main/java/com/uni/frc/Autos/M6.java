package com.uni.frc.Autos;

import java.util.Arrays;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.subsystems.Shooter;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import com.uni.lib.motion.PathStateGenerator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class M6 extends AutoBase {
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive mSwerve = SwerveDrive.getInstance();
    double initRotation = 0;
    PathPlannerPath path = PathPlannerPath.fromPathFile("M6 KALMAN");
    PathPlannerTrajectory trajectory = path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation));

    @Override
    public void auto() {
        Shooter.getInstance().setPercent(0.8);
        //Shot 1
        PathStateGenerator.getInstance().setTrajectory(trajectory);
        s.setPivotState(0.083-.125);
        s.shootState(false);
        s.trajectoryState(trajectory, initRotation);
        registerTrajectoryStops(Arrays.asList(1.23,2.83,4.1,7.6,10.76));

        s.waitForPositionState(0.78);
        s.intakeState(.3, false);
        s.preparePivotState();
        s.shootState(false);
        s.resumeTrajectoryState();

        s.waitForPositionState(2.6);
        s.intakeState(.3,false);
        s.preparePivotState();
        s.shootState(false);
        s.resumeTrajectoryState();

        s.waitForPositionState(3.9);
        s.intakeState(.3, false);
        s.setPivotState(-0.165);

        s.shootState(false);
        s.resumeTrajectoryState();

        s.waitForPositionState(5);

        s.intakeState(1.5,false);

        s.waitForPositionState(7);
        s.setPivotState(0.083-0.285);
        s.waitForPositionState(7.6);
        s.shootState(false);
        s.resumeTrajectoryState();

    
        s.waitForPositionState(8.3);
        s.intakeState(1.5,false);
        s.waitForPositionState(10);
        s.setPivotState(0.083-0.292);

        s.waitForPositionState(10.83);
        s.shootState(false);
        s.resumeTrajectoryState();
    }

}