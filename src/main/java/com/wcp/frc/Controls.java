// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc;

import java.util.Optional;

import com.wcp.frc.Planners.AutoAlignPointSelector;
import com.wcp.frc.subsystems.RobotState;
import com.wcp.frc.subsystems.SuperStructure;
import com.wcp.frc.subsystems.Swerve.SwerveDrive;
import com.wcp.frc.subsystems.Swerve.SwerveDrive.State;
import com.wcp.lib.geometry.Pose2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class Controls {
    SuperStructure s;
    XboxController Driver;
    XboxController CoDriver;
    SwerveDrive swerve;

    ButtonCheck driverLeftTrigger = new ButtonCheck(.5);
    ButtonCheck driverRightTrigger = new ButtonCheck(.5);
    ButtonCheck driverLeftBumper = new ButtonCheck();
    ButtonCheck driverRightBumper = new ButtonCheck();
    ButtonCheck driverLeftStick = new ButtonCheck();
    ButtonCheck driverAButton = new ButtonCheck();
    ButtonCheck driverXButton = new ButtonCheck();
    ButtonCheck driverYButton = new ButtonCheck();
    ButtonCheck driverBButton = new ButtonCheck();
    ButtonCheck driverRightStickDown = new ButtonCheck();
    ButtonCheck driverStartButton = new ButtonCheck();
    ButtonCheck driverDpadLeft = new ButtonCheck();
    ButtonCheck driverDpadUp = new ButtonCheck();
    ButtonCheck driverDpadRight = new ButtonCheck();
    ButtonCheck driverDpadDown = new ButtonCheck();
    ButtonCheck driverBackButtonDown = new ButtonCheck();

    ButtonCheck coDriverStart = new ButtonCheck();
    ButtonCheck coDriverAButton = new ButtonCheck();
    ButtonCheck coDriverBButton = new ButtonCheck();
    ButtonCheck coDriverYButton = new ButtonCheck();
    ButtonCheck coDriverXButton = new ButtonCheck();
    ButtonCheck codriverLeftBumper = new ButtonCheck();
    ButtonCheck codriverRightBumper = new ButtonCheck();
    ButtonCheck coDriverLeftTrigger = new ButtonCheck(.5);
    ButtonCheck coDriverRightTrigger = new ButtonCheck(.5);
    ButtonCheck coDriverLeftStickDown = new ButtonCheck();
    ButtonCheck coDriverRightStickDown = new ButtonCheck();
    ButtonCheck coDriverBackButton = new ButtonCheck();

    private static Controls instance = null;

    public static Controls getInstance() {
        if (instance == null)
            instance = new Controls();
        return instance;
    }

    public Controls() {
        Driver = new XboxController(Ports.XBOX_1);
        CoDriver = new XboxController(Ports.XBOX_2);
        swerve = SwerveDrive.getInstance();
    }

    public void update() {
        var timestamp = Timer.getFPGATimestamp();
        s = SuperStructure.getInstance();

        double driverLeftXInput = Driver.getLeftX();
        double driverLeftYInput = Driver.getLeftY();
        double driverRightXInput = Driver.getRightX();

        double coDriverLeftX = CoDriver.getLeftX();
        double coDriverLeftY = CoDriver.getLeftY();
        double coDriverRightY = CoDriver.getRightY();
        double coDriverRightX = CoDriver.getRightX();

        driverLeftTrigger.update(Driver.getLeftTriggerAxis());
        driverRightTrigger.update(Driver.getRightTriggerAxis());
        driverLeftBumper.update(Driver.getLeftBumper());
        driverRightBumper.update(Driver.getRightBumper());
        driverLeftStick.update(Driver.getLeftStickButton());
        driverAButton.update(Driver.getAButton());
        driverXButton.update(Driver.getXButton());
        driverYButton.update(Driver.getYButton());
        driverBButton.update(Driver.getBButton());
        driverRightStickDown.update(Driver.getRightStickButton());
        driverStartButton.update(Driver.getStartButton());
        driverDpadLeft.update(Driver.getPOV() == 270);
        driverDpadUp.update(Driver.getPOV() == 0);
        driverDpadRight.update(Driver.getPOV() == 90);
        driverDpadDown.update(Driver.getPOV() == 180);

        coDriverStart.update(CoDriver.getStartButton());
        coDriverAButton.update(CoDriver.getAButton());
        coDriverBButton.update(CoDriver.getBButton());
        coDriverYButton.update(CoDriver.getYButton());
        coDriverXButton.update(CoDriver.getXButton());
        codriverLeftBumper.update(CoDriver.getLeftBumper());
        codriverRightBumper.update(CoDriver.getRightBumper());
        coDriverLeftTrigger.update(CoDriver.getLeftTriggerAxis());
        coDriverRightTrigger.update(CoDriver.getRightTriggerAxis());
        coDriverLeftStickDown.update(CoDriver.getLeftStickButton());
        coDriverRightStickDown.update(CoDriver.getLeftStickButtonPressed());
        coDriverBackButton.update(CoDriver.getBackButton());

        if (driverStartButton.isPressed())
            swerve.resetGryo(180);

        if (driverLeftTrigger.getValue() > 0.2) {
            swerve.sendInput(-driverLeftYInput, driverLeftXInput, -driverRightXInput, State.AIMING);
        } else if (driverRightTrigger.getValue() > .2) {
            Optional<Pose2d> targetSnap = AutoAlignPointSelector
                    .chooseTargetPoint(RobotState.getInstance().getAbosoluteKalmanPose(timestamp));
            if (targetSnap.isEmpty()) {
            swerve.sendInput(-driverLeftYInput, driverLeftXInput, -driverRightXInput, State.MANUAL);
            } else {
                swerve.setAlignment(targetSnap.get());
                swerve.sendInput(-driverLeftYInput, driverLeftXInput, -driverRightXInput, State.ALIGNMENT);
            }
        } else {
            swerve.sendInput(-driverLeftYInput, driverLeftXInput, -driverRightXInput, State.MANUAL);
        }

    }

    public class ButtonCheck {
        double value;
        double threshold;
        Boolean[] input = new Boolean[2];

        public ButtonCheck(double threshold) {
            this.threshold = threshold;
            for (int i = 0; i < 1; i++) {
                input[i] = false;
            }
        }

        public ButtonCheck() {
            this.threshold = 0;
            for (int i = 0; i < 1; i++) {
                input[i] = false;
            }
        }

        public void update(double input) {
            value = input;
        }

        public void update(boolean input) {
            this.input[1] = this.input[0];
            if (input)
                this.input[0] = true;
            else
                this.input[0] = false;
        }

        public boolean isPressed() {
            return input[0] && !input[1];
        }

        public double getValue() {
            return value;
        }

        public boolean isReleased() {
            return !input[0] && input[1];
        }

        public boolean isActive() {
            return input[0];
        }

        public void setThreshhold(double threshold) {
            this.threshold = threshold;
        }
    }
}
