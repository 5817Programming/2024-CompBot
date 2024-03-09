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

public class S1 extends AutoBase {
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive mSwerve = SwerveDrive.getInstance();
    double initRotation = 0;
    PathPlannerPath path = PathPlannerPath.fromPathFile("S1");
    PathPlannerTrajectory trajectory = path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation));

    @Override
    public void auto() {
        Shooter.getInstance().setPercent(0.8);
        //Shot 1
        PathStateGenerator.getInstance().setTrajectory(trajectory);
        registerTrajectoryStops(Arrays.asList(0.78));


        s.trajectoryState(trajectory, initRotation);


        s.waitForPositionState(0.78);
        s.preparePivotState();
        s.shootState(false);
        s.resumeTrajectoryState();

      
    }

}