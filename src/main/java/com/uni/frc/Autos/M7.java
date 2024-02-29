package com.uni.frc.Autos;



import java.util.List;

import org.ejml.dense.row.CommonOps_MT_CDRM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.Swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;


public class M7 extends AutoBase{
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive mSwerve = SwerveDrive.getInstance();
    double initRotation = -2;
    PathPlannerPath path = PathPlannerPath.fromPathFile("M7 SPEED");
    PathPlannerTrajectory trajectory = path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation));
    List<EventMarker> eventMarkers = path.getEventMarkers();

    private List<EventMarker> events;
    private int event = 0;

    @Override
    public void auto(){
        registerEventMarkers(eventMarkers);
        s.trajectoryState(trajectory, initRotation);
        s.shootState(false);
        s.waitForEventState(0, events);
        s.intakeState(false);
        // eventWaitState();
        // s.shootState(false);
        // eventWaitState();

        // eventWaitState();
        // s.intakeState(false);
        // eventWaitState();
        // s.shootState(false);

        // eventWaitState();
        // s.intakeState(false);
        // eventWaitState();
        // s.shootState(false);
        
        // eventWaitState();
        // s.intakeState(false);
        // eventWaitState();
        // s.shootState(false);

        // eventWaitState();
        // s.intakeState(false);
        // eventWaitState();
        // s.shootState(false);

        // eventWaitState();
        // s.intakeState(false);
        // eventWaitState();
        // s.shootState(false);

        // s.waitState(25, false);



    }

    public void registerEventMarkers(List<EventMarker> markers){
        events = markers;
    }

    public void eventWaitState(){
        event += 1;
    }

}