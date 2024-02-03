// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.Planners;

import java.util.Optional;
import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

import com.wcp.frc.Constants;
import com.wcp.frc.subsystems.RobotState;
import com.wcp.frc.subsystems.Requests.Request;
import com.wcp.frc.subsystems.Vision.ObjectLimeLight.VisionObjectUpdate;
import com.wcp.frc.subsystems.Vision.OdometryLimeLight.VisionUpdate;
import com.wcp.frc.subsystems.gyros.Pigeon;
import com.wcp.lib.HeadingController;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;

import com.wcp.lib.util.InterpolatingDouble;
import com.wcp.lib.util.InterpolatingTreeMap;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class TargetPiecePlanner {

    public double updateAiming(double timeStamp,Optional<VisionObjectUpdate> visionupdate, HeadingController headingController, Rotation2d currentHeading){
        if(visionupdate.isEmpty()){
            System.out.println("visionempty");
            return 0;
        }
        double objectYDegreesCamera = visionupdate.get().getCameraToTarget().x();
        Rotation2d targetHeading = currentHeading.rotateBy(objectYDegreesCamera);
        Logger.recordOutput("objectTarget", targetHeading.getDegrees());
        headingController.setTargetHeading(targetHeading);
        double output = headingController.getRotationCorrection(currentHeading, timeStamp);   
        return output;
    }


}
