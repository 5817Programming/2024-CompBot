// package com.uni.lib.util;
 

//  import edu.wpi.first.wpilibj.Timer;

//  import org.littletonrobotics.junction.Logger;

// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.path.PathPoint;
// import com.pathplanner.lib.path.RotationTarget;
// import com.uni.lib.geometry.Pose2d;
// import com.uni.lib.geometry.Rotation2d;
// import com.uni.lib.geometry.Translation2d;
// import com.uni.lib.geometry.HeavilyInspired.Edge;
// import com.uni.lib.geometry.HeavilyInspired.Node;
// import com.uni.lib.geometry.HeavilyInspired.Obstacle;
// import com.uni.lib.geometry.HeavilyInspired.VisGraph;

// import java.util.ArrayList;
//  import java.util.List;

//  /** Custom PathPlanner version of SwerveControllerCommand */
//  public class PathGenerator {
//      public static boolean  ran = false;

//      public static PathPlannerTrajectory generatePath(Pose2d currentPose, PathConstraints constraints,Node endTarget, List<Obstacle> obstacles, Rotation2d endRotation){
//          double grain = 3.5;

//          List<Node> fullPath = new ArrayList<Node>();
//          Node start = new Node(currentPose);
//          VisGraph aStar = VisGraph.getInstance();
//          List<PathPoint> fullPathPoints = new ArrayList<PathPoint>();
//          List<Obstacle> totalObstacles = new ArrayList<>();
  
//         for(int i = 0; i < obstacles.size(); i ++){
//                         totalObstacles.add(obstacles.get(i));
//                     }

//      aStar.addNode(endTarget);

//          if(!ran){
//          for(double i = 2*grain; i < 15*grain; i++){
//              for( double j = 0; j < 9*grain; j++){
//                  aStar.addNode(new Node(i/grain,j/grain));
//              }
//          }
//          }

//      for(int i = 0; i<aStar.getNodeSize();i++){
//        Node startNode = aStar.getNode(i);
//        for(int j = i; j<aStar.getNodeSize(); j++){
//          aStar.addEdge(new Edge(startNode, aStar.getNode(j)), totalObstacles);
//        }
//      }
    

//          if(aStar.addEdge(new Edge(start, endTarget), totalObstacles)){
//              fullPath.add(0,start);
//              fullPath.add(1,endTarget);

//          }else{
//              for(int i = 0; i<aStar.getNodeSize();i++){
//                  Node end = aStar.getNode(i);
//                  aStar.addEdge(new Edge(start,end), totalObstacles);
//              }
//              fullPath = aStar.findPath(start, endTarget);
//          }

//          if(fullPath != null){
//              edu.wpi.first.math.geometry.Rotation2d Heading = new Rotation2d(fullPath.get(1).getX()-start.getX(),fullPath.get(1).getY()-start.getY()).toWPI();

//          for(int i = 0; i < fullPath.size(); i++){
//              if(i == 0){
//                  fullPathPoints.add(new PathPoint(start.getTranslation().toWPI(), Heading ,start.getHolRot()));
//              }
//              else if(i + 1 == fullPath.size()){
//                  edu.wpi.first.math.geometry.Translation2d translation = new Translation2d(endTarget.getTranslation()).toWPI();
//                  edu.wpi.first.math.geometry.Rotation2d heading = new Rotation2d(fullPath.get(i).getX() - fullPath.get(i - 1).getX(), endTarget.getY() - fullPath.get(i - 1).getY()).toWPI();
//                  fullPathPoints.add(i,new PathPoint(translation,new RotationTarget(i,heading,true)));
//              }
//              else if(i < fullPath.size() - 3){
//                  edu.wpi.first.math.geometry.Translation2d translation = new Translation2d(fullPath.get(i).getTranslation().getX(),fullPath.get(i).getTranslation().getY()).toWPI();
//                  edu.wpi.first.math.geometry.Rotation2d heading = fullPath.get(i+1).getTranslation().translateBy(fullPath.get(i).getTranslation().inverse()).getAngle().toWPI();
//                  fullPathPoints.add(i,new PathPoint(translation, heading, endTarget.getHolRot(),5).withControlLengths(.5,.2));
//              }else{
//                  edu.wpi.first.math.geometry.Translation2d translation = new Translation2d(fullPath.get(i).getTranslation().getX(),fullPath.get(i).getTranslation().getY()).toWPI();
//                  edu.wpi.first.math.geometry.Rotation2d heading = fullPath.get(i+1).getTranslation().translateBy(fullPath.get(i).getTranslation().inverse()).getAngle().toWPI();
//                  fullPathPoints.add(i,new PathPoint(translation, heading, endTarget.getHolRot()).withControlLengths(.3,.3));
//              }
            
//          }
//          ran = true;
 
//          Logger.recordOutput("desiredPose",PathPlanner.generatePath(constraints,fullPathPoints).sample(Timer.getFPGATimestamp()%PathPlanner.generatePath(constraints,fullPathPoints).getTotalTimeSeconds()).poseMeters);
//          return PathPlannerPath.fromPathPoints(constraints,fullPathPoints,new GoalEndState(0, endRotation.to));
//          }else{
//              return new PathPlannerTrajectory();
//          }
        
//      }
    
//  }
