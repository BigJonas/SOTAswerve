// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Swerve.SwerveDrive;

/**
 * Drives the swerve while rotating the swerve to a provieded angle
 */
public class RotateSwerveToDrive extends DefaultDrive {
  /** Creates a new RotateSwerveTo. */
  public RotateSwerveToDrive(SwerveDrive swerve, DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier desiredAngle) {
    super(
      swerve, 
      fwd, 
      str, 
      () -> {
        Rotation2d desiredRotation = new Rotation2d(desiredAngle.getAsDouble());
        Rotation2d currentRotation = swerve.getRotation2d();
        double error = desiredRotation.minus(swerve.getRotation2d()).getRadians();
        if (error > Math.PI) { // If the error is greater than 180 degrees than turn other direction 
          error = -1 * (Rotation2d.fromRadians(2 * Math.PI).minus(desiredRotation)).plus(currentRotation).getRadians();
        }
        return swerve.getRotationTranslationController().calculate(error, 0.0); // Tries to get error to zero
      }
    );    
  }
}
