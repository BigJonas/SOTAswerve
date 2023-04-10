// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class DefaultDrive extends CommandBase {
  private final SwerveDrive mSwerve;
  private final DoubleSupplier mFwdSupplier;
  private final DoubleSupplier mStrSupplier;
  private final DoubleSupplier mRotSupplier;

  /** Creates a new DefaultDrive. */
  public DefaultDrive(SwerveDrive swerve, DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rot) {
    // Use addRequirements() here to declare subsystem dependencies.
    mSwerve = swerve;

    mFwdSupplier = fwd;
    mStrSupplier = str;
    mRotSupplier = rot;

    addRequirements(mSwerve);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double fwd = mFwdSupplier.getAsDouble();
    double str = mStrSupplier.getAsDouble();
    double rot = mRotSupplier.getAsDouble();

    drive(fwd, str, rot);
  }

  protected void drive(double fwd, double str, double rot) {
    mSwerve.drive(fwd, str, rot);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
