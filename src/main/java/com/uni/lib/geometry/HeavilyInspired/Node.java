package com.uni.lib.geometry.HeavilyInspired;

import java.util.ArrayList;
import java.util.List;

import com.uni.frc.subsystems.Swerve.SwerveDrive;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Translation2d;


public class Node {
    public double x;
    public double y;
    Rotation2d holonomicRotation;
    public List < Node > neighbors;
  
    public Node(double x, double y) {
        this.x = x;
        this.y = y;
        holonomicRotation = Rotation2d.fromDegrees(0);
        this.neighbors = new ArrayList < > ();
    }
  
    public Node(double x, double y, Rotation2d holonomicRotation) {
        this.x = x;
        this.y = y;
        this.holonomicRotation = holonomicRotation;
        this.neighbors = new ArrayList < > ();
    }
    public Node(Translation2d coordinates, Rotation2d holonomicRotation) {
      this.x = coordinates.x();
      this.y = coordinates.y(); 
      this.holonomicRotation = holonomicRotation;
      this.neighbors = new ArrayList < > ();
    }



	public void addNeighbor(Node neighbor) {
        this.neighbors.add(neighbor);
    }
    public double getX(){
      return x;
    }
    public double getY(){
      return y;
    }
    public Translation2d getTranslation(){
      return new Translation2d(x,y);
    }
    public Rotation2d getHolRot(){
      return holonomicRotation;
    }
    public void setHolRot(double degree){
      this.holonomicRotation = Rotation2d.fromDegrees(degree);
    } 
  }
