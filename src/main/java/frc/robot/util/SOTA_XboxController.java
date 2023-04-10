// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class SOTA_XboxController implements SOTA_Controller {
    private CommandXboxController mController;
    private boolean mSquareInputs;

    // top 10 boilerplate code moments
    public SOTA_XboxController(int port) {
        this(port, true);
    }

    public SOTA_XboxController(int port, boolean squareInputs) {
        mController = new CommandXboxController(port);
        mSquareInputs = squareInputs;
    }

    public boolean getSelect() {
        return mController.back().getAsBoolean();
    }

    public boolean getStart() {
        return mController.start().getAsBoolean();
    }

    public boolean getA() {
        return mController.a().getAsBoolean();
    }

    public boolean getB() {
        return mController.b().getAsBoolean();
    }

    public boolean getX() {
        return mController.x().getAsBoolean();
    }

    public boolean getY() {
        return mController.y().getAsBoolean();
    }

    public boolean getLeftBumper() {
        return mController.leftBumper().getAsBoolean();
    }

    public boolean getLeftTrigger() {
        return mController.leftTrigger().getAsBoolean();
    }

    public boolean getRightBumper() {
        return mController.rightBumper().getAsBoolean();
    }

    public boolean getRightTrigger() {
        return mController.rightTrigger().getAsBoolean();
    }

    public double getLeftTriggerAxis() {
        return mController.getLeftTriggerAxis();
    }

    public double getRightTriggerAxis() {
        return mController.getRightTriggerAxis();
    }

    public double getLeftStickX() {
        double input = deadband(mController.getLeftX());
        if (mSquareInputs) input = square(input);
        return input;
    }

    public double getLeftStickY() {
        double input = deadband(mController.getLeftY());
        if (mSquareInputs) input = square(input);
        return input;
    }

    public double getRightStickX() {
        double input = deadband(mController.getRightX());
        if (mSquareInputs) input = square(input);
        return input;
    }

    public double getRightStickY() {
        double input = deadband(mController.getRightY());
        if (mSquareInputs) input = square(input);
        return input;
    }

    public Trigger select() {
        return mController.back();
    }

    public Trigger start() {
        return mController.start();
    }

    public Trigger a() {
        return mController.a();
    }

    public Trigger b() {
        return mController.b();
    }

    public Trigger x() {
        return mController.x();
    }

    public Trigger y() {
        return mController.y();
    }

    public Trigger leftBumper() {
        return mController.leftBumper();
    }

    public Trigger leftTrigger() {
        return mController.leftTrigger();
    }

    public Trigger rightBumper() {
        return mController.rightBumper();
    }

    public Trigger rightTrigger() {
        return mController.rightTrigger();
    }

    public Trigger leftStick() {
        return mController.leftStick();
    }

    public Trigger rightStick() {
        return mController.rightStick();
    }

}
