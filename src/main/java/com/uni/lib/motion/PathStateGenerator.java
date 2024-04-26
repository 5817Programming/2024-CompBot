package com.uni.lib.motion;

import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.lib.geometry.Pose2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class PathStateGenerator{

  public static PathStateGenerator instance = null;

  private Timer mTimer;
  private boolean useAlliance;
  private Optional<PathPlannerTrajectory> trajectory;

  public static PathStateGenerator getInstance(){
    if (instance == null) 
      instance = new PathStateGenerator();
    return instance;
  }

  private PathStateGenerator(){
    mTimer = new Timer();
    trajectory = Optional.empty();
    useAlliance = true;
  }

  public void setTrajectory(PathPlannerTrajectory newTrajectory){
    trajectory = Optional.of(newTrajectory); 
  }

  public void resetTrajectory(){
    mTimer.reset();
    mTimer.stop();

    trajectory = Optional.empty();
  }

  public void useAlliance(boolean use){
    useAlliance = use;
  }

  public void stopTrajectory(){
    mTimer.stop();
  }
  
  public void startTrajectory(){
    mTimer.start();
  }

  public Pose2d getInitialPose(){
    if(trajectory.isEmpty())
      return Pose2d.identity();
    return sample(1E-20).get();
  }

  public Optional<Pose2d> sample(double timestamp){
    if(trajectory.isEmpty())
        return Optional.empty();
    if(useAlliance && DriverStation.getAlliance().get() == Alliance.Red)
        return Optional.of(new Pose2d(trajectory.get().sample(timestamp).getTargetHolonomicPose()).mirror());
    return Optional.of(new Pose2d(trajectory.get().sample(timestamp).getTargetHolonomicPose()));
  }
  
  public Optional<Pose2d> getDesiredPose2d(){
    if(trajectory.isEmpty())
        return Optional.empty();
    if(useAlliance && DriverStation.getAlliance().get() == Alliance.Red)
        return Optional.of(new Pose2d(trajectory.get().sample(mTimer.get()).getTargetHolonomicPose()).mirror());
    return Optional.of(new Pose2d(trajectory.get().sample(mTimer.get()).getTargetHolonomicPose()));
  }




}