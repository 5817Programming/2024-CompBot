// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import org.littletonrobotics.junction.Logger;

import com.wcp.frc.Constants;
import com.wcp.frc.subsystems.Requests.Request;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends Subsystem {
  public static Vision instance = null;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry botpose = table.getEntry("botpose");


  public static Vision getInstance() {
    if (instance == null)
      instance = new Vision();
    return instance;
  }

  public Vision() {


  }

  public double getX() {
    return tx.getDouble(0.0);//gets limelight x
  }

  public double getY() {
    return ty.getDouble(0.0);//gets limelight y
  }

  public double getArea() {
    return ta.getDouble(0.0);//gets limelight area
  }


  public double getYaw() {
    return tx.getDouble(0.0);
  }
  public boolean hasTarget() {//returns in binary so we convert to boolean 
    double v = tv.getDouble(0.0);
    if (v == 0.0f) {
      return false;
    } else {
      return true;
    }
  }

  public double getDistance(){//gets distance to target
    double distanceFromLimelightToGoalInches = (Constants.VisionConstants.APRIL_HEIGHT_INCHES - Constants.VisionConstants.LIMELIGHT_LENS_HEIGHT_INCHES)/Math.tan(Math.toRadians(Constants.VisionConstants.LIMELIGHT_MOUNT_ANGLE_DEGREES + getY()));
  return distanceFromLimelightToGoalInches>0&&distanceFromLimelightToGoalInches<1000?Units.inchesToMeters(distanceFromLimelightToGoalInches):0;
  }

  public double getDistanceToGroundObject(){//gets distance to target
    double distanceFromLimelightToGoalInches = (0 - Constants.VisionConstants.LIMELIGHT_LENS_HEIGHT_INCHES)/Math.tan(Math.toRadians(Constants.VisionConstants.LIMELIGHT_MOUNT_ANGLE_DEGREES + getY()));
  return distanceFromLimelightToGoalInches>0&&distanceFromLimelightToGoalInches<1000?Units.inchesToMeters(distanceFromLimelightToGoalInches):0;
  }

  public void setPipeline(Integer pipeline) {
    if(pipeline<0){
        pipeline = 0;
        throw new IllegalArgumentException("Pipeline can not be less than zero");
    }else if(pipeline>9){
        pipeline = 9;
        throw new IllegalArgumentException("Pipeline can not be greater than nine");
    }
    table.getEntry("pipeline").setValue(pipeline);
  }
  public Request pipleLineRequest(int pipeline){
    return new Request () {
      public void act() {
          setPipeline(pipeline);
      }
    };
  }

  @Override
  public void update(){
  }

  @Override
  public void outputTelemetry() {
    Logger.recordOutput("Distance", getDistance());
    Logger.recordOutput("hasTarget", hasTarget());
    Logger.recordOutput("tx", tx.getDouble(0.0));
    Logger.recordOutput("ty", ty.getDouble(0.0));
    Logger.recordOutput("ta", ta.getDouble(0.0));
  }
  

  @Override
  public void stop() {
    
  }



}
