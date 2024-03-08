// package com.uni.frc.Autos;

// import java.util.Arrays;

// import com.fasterxml.jackson.core.util.DefaultPrettyPrinter.Indenter;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.uni.frc.subsystems.Indexer;
// import com.uni.frc.subsystems.Intake;
// import com.uni.frc.subsystems.Shooter;
// import com.uni.frc.subsystems.SuperStructure;
// import com.uni.frc.subsystems.Swerve.SwerveDrive;
// import com.uni.lib.geometry.Pose2d;
// import com.uni.lib.geometry.Translation2d;
// import com.uni.lib.motion.PathStateGenerator;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;

// public class S4 extends AutoBase {
//     SuperStructure s = SuperStructure.getInstance();
//     SwerveDrive mSwerve = SwerveDrive.getInstance();
//     double initRotation = -60;
//     PathPlannerPath path = PathPlannerPath.fromPathFile("S4");
//     PathPlannerTrajectory trajectory = path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation));

//     @Override
//     public void auto() {
//         Shooter.getInstance().setPercent(0.8);
//         //Shot 1
//         PathStateGenerator.getInstance().setTrajectory(trajectory);
//         s.setPivotState(-.179);
//         s.shootState(false);
//         s.trajectoryState(trajectory, initRotation);
//         registerTrajectoryStops(Arrays.asList(4.5,8.12,11.7));


//         s.waitForPositionState(2.1);
//         s.intakeState(1.5);
//         s.waitForPositionState(3.5);
//         s.setPivotState(-.33);
//         s.waitForPositionState(4.5);
//         s.shootState(false);
//         s.resumeTrajectoryState();

//         s.waitForPositionState(5.7);
//         s.intakeState(1.5);
//         s.waitForPositionState(7.24);
//         s.setPivotState(-.33);
//         s.waitForPositionState(8.12);
//         s.shootState(false);
//         s.resumeTrajectoryState();

//         s.waitForPositionState(9.4);
//         s.intakeState(1.5);
//         s.waitForPositionState(10.82);
//         s.setPivotState(-.33);
//         s.waitForPositionState(11.7);
//         s.shootState(false);
//         s.resumeTrajectoryState();
//     }

// }