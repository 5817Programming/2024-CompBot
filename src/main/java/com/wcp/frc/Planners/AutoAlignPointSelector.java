// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.Planners;

import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.wcp.frc.Field.AprilTag;
import com.wcp.frc.Field.FieldLayout;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class AutoAlignPointSelector {

    private static Map<Integer,AprilTag> getTagSet(){
        if(DriverStation.getAlliance().get() ==  Alliance.Red){
            return FieldLayout.Red.kAprilTagMap;
        } else{
            return FieldLayout.Blue.kAprilTagMap;
        }
    }

    private static Optional<AprilTag> getNearestTag(Map<Integer, AprilTag> TagMap, Pose2d point){
        Logger.recordOutput("jagklawgj", point.toWPI()); 
        double closestDistance = Integer.MAX_VALUE;
        Optional<AprilTag> closestTag = Optional.empty();
        for(int i:TagMap.keySet()){
            AprilTag currentTag = TagMap.get(i);
            double distance = currentTag.getFieldToTag().transformBy((currentTag.getTagToCenterAlign())).distance(point);
            if(distance < closestDistance){
                closestDistance = distance;
                closestTag = Optional.of(TagMap.get(i));
            }
        }
        return closestTag;
    }

    private static Optional<Pose2d> minimizeDistance(Pose2d from, Pose2d[]to){//Unused 
        if(to.length == 0){
            return Optional.empty();
        }
        double closestDistance = Integer.MAX_VALUE;
        Pose2d closestPose = to[0];
        for(int i = 0; i < to.length; i++){
            double distance = from.distance(to[i]);
            if(distance < closestDistance){
                closestDistance = distance;
                closestPose = to[i];
            }
        }

        return Optional.of(closestPose);
    }

    private static Optional<Pose2d> getNearestAlignment(AprilTag tag, Pose2d point) {//TODO set the rotation 2ds to be perpendicular to the apriltags
        if (tag.isScoring()) {
            Pose2d center = tag.getFieldToTag().transformBy((tag.getTagToCenterAlign()));
            return Optional.of(center);
        } else {
            Pose2d left = tag.getFieldToTag().transformBy((tag.getTagToLeftAlign()));
            Pose2d right = tag.getFieldToTag().transformBy((tag.getTagToRightAlign()));
            Pose2d center = tag.getFieldToTag().transformBy((tag.getTagToCenterAlign()));
            return minimizeDistance(point, new Pose2d[]{left,right,center});
        }
    }

    public static Optional<Pose2d> chooseTargetPoint(Pose2d point){
        Map<Integer, AprilTag> mTagMap = getTagSet();
        Optional<AprilTag> closestAprilTag = getNearestTag(mTagMap, point);
        Optional<Pose2d> targetPose = Optional.empty(); 
        Logger.recordOutput("clsest tag id", closestAprilTag.get().getId());
        targetPose = getNearestAlignment(closestAprilTag.get(), point);

        if(!targetPose.isPresent()){
            return Optional.empty();
        }
        return targetPose;
    }
}
