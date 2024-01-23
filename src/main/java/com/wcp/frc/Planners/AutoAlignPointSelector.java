// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.Planners;

/** Add your docs here. */
public class AutoAlignPointSelector {
    private static final Pose2d[] kposeSetpoints = Pose2d[new Pose2d]; 

    public enum AlignmentRequest{
        April,
        Point,
    }

    private static Map<Integer,AprilTag> getTagSet(){
        if(DriverStation.getAlliance().getValue ==  Alliance.Red){
            return FieldLayout.Red.kAprilTagMap;
        } else{
            return FieldLayout.Blue.kAprilTagMap;
        }
    }

    private static AprilTag getNearestTag(Map<Integer, AprilTag> TagMap, Pose2d point){
        double closestDistance = Integer.MAX_VALUE;
        Optional<AprilTag> closestTag = Optional.empty();
        for(int i:TagMap.keySet()){
            AprilTag currentTag = TagMap.get(i);
            double distance = currentTag.getFieldToTag().transformBy(Pose2d.fromTranslation(currentTag.getTagToCenterAlign)).distance(point);
            if(distance < closestDistance){
                closestDistance = distance;
                closestTag = TagMap.get(i);
            }
        }
        return closestTag
    }

    private static Optional<Pose2d> minimizeDistance(Pose2d from, Pose2d[]to){
        if(to.size() == 0){
            return Optional.empty()
        }
        double closestDistance = Integer.MAX_VALUE;
        Pose2d closestPose = to[0];
        for(int i = 0; i < to.size(); i++){
            double distance = from.distace(to[i]);
            if(distace < closestDistance){
                closestDistance = distance;
                closestPose = to[i];
            }
        }

        return Optional.of(closestPose);
    }

    private static Optional<Pose2d> chooseTargetPoint(Pose2d point, AlignmentRequest alignmentRequest){
        switch(alignmentRequest)
            case AlignmentRequest.April:
                Map<Integer, AprilTag> mTagMap = getTagSet();
                AprilTag closestAprilTag = getNearestTag(mTagMap, point);
                return Optional.of(closestAprilTag.getFieldToTag().transformBy(closestAprilTag.getTagToCenterAlign()));
            break;
            case AlignmentRequest.Point:
                Pose2d closestPoint = minimizeDistance(point, poseSetpoints);
                return closestPoint;
            break;
        
        return Optional.empty();

    }
}
