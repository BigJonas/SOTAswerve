// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class FlickStickDrive extends RotateSwerveToDrive {
  /** Creates a new FlickStickDrive. */
  public FlickStickDrive(SwerveDrive swerve, DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier desiredAngle, DoubleSupplier anglePower) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
      swerve,
      fwd,
      str,
      () -> {
        if (anglePower.getAsDouble() <= Math.sqrt(2) * 0.01) return swerve.getYaw(); // If power is less than 1 percent of max turn to current angle
        return desiredAngle.getAsDouble();
      }
    );
  }
}
