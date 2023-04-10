// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public interface SOTA_Controller {

    public boolean getSelect();
    public boolean getStart();
    public boolean getA();
    public boolean getB();
    public boolean getX();
    public boolean getY();
    public boolean getLeftBumper();
    public boolean getLeftTrigger();
    public boolean getRightBumper();
    public boolean getRightTrigger();
    public double getLeftTriggerAxis();
    public double getRightTriggerAxis();
    public double getLeftStickX();
    public double getLeftStickY();
    default double getLeftStickAngle() {
        double x = getLeftStickX();
        double y = -getLeftStickY();
        return Math.atan2(y, x);
    }
    default Rotation2d getLeftStickRotation() {
        return new Rotation2d(getLeftStickAngle());
    }
    default double getLeftStickPower() {
        double x = getLeftStickX();
        double y = getLeftStickY();
        return Math.sqrt((x * x) + (y * y));
    }
    public double getRightStickX();
    public double getRightStickY();
    default double getRightStickAngle() {
        double x = getRightStickX();
        double y = -getRightStickY();
        return Math.atan2(y, x);
    }
    default Rotation2d getRightStickRotation() {
        return new Rotation2d(getLeftStickAngle());
    }
    default double getRightStickPower() {
        double x = getRightStickX();
        double y = getRightStickY();
        return Math.sqrt((x * x) + (y * y));
    }
    public Trigger select();
    public Trigger start();
    public Trigger a();
    public Trigger b();
    public Trigger x();
    public Trigger y();
    public Trigger leftBumper();
    public Trigger leftTrigger();
    public Trigger rightBumper();
    public Trigger rightTrigger();
    public Trigger leftStick();
    public Trigger rightStick();        
    default double deadband(double input) {
        return Math.abs(input < 0.1 ? 0 : input);
    }
    default double square(double input) {
        return Math.signum(input) * input * input;
    }
}
