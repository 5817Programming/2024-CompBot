// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib;


import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.util.SynchronousPIDF;

/** Add your docs here. */
public class HeadingController {
    private Rotation2d targetHeading = Rotation2d.fromDegrees(0);
    private double lastTimestamp;

    private double disabledStartTimestamp = 0;
    private boolean isDisabled = false;
    public void disableHeadingController(boolean disable) {
        disabledStartTimestamp = lastTimestamp;
        isDisabled = disable;
    }

    private SynchronousPIDF pidController;
    public HeadingController() {
        pidController = new SynchronousPIDF(0.0003, 0, 0, 0.0);
    }
    public void setTargetHeading(Rotation2d heading) {
        targetHeading = Rotation2d.fromDegrees(heading.getDegrees());
    }
    public double updateRotationCorrection(Rotation2d heading, double timestamp) {
        if(isDisabled) {
            if((timestamp - disabledStartTimestamp) > 0.25) {
                isDisabled = false;
                setTargetHeading(heading);
            }
            return 0;
        }
        return getRotationCorrection(heading, timestamp);
    }
    public double getRotationCorrection(Rotation2d heading, double timestamp) {
        double error = new Rotation2d(targetHeading).rotateBy(heading.inverse()).getDegrees();
        double dt = timestamp - lastTimestamp;
        lastTimestamp = timestamp;
        double correctionForce = pidController.calculate(error, dt);
        if(Math.abs(correctionForce) > 0.017){
            correctionForce = .017* Math.signum(correctionForce);
        }
        return correctionForce;
    }

}
