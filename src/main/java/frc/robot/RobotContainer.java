// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.FlickStickDrive;
import frc.robot.commands.RotateSwerveToDrive;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.SOTA_Controller;
import frc.robot.util.SOTA_XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrive mSwerveDrive;

  private final DefaultDrive mDefaultDrive;
  private final FlickStickDrive mFlickStickDrive;

  private final SOTA_Controller mDriverController =
    new SOTA_XboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    mSwerveDrive = new SwerveDrive(null, null, null); // TODO: make a swerve module factory or something silly like that

    // Examples of default drive and flick stick drive implementations 
    mDefaultDrive = new DefaultDrive(
      mSwerveDrive, 
      () -> mDriverController.getLeftStickY(),
      () -> mDriverController.getLeftStickX(),
      () -> mDriverController.getRightStickX()
    );
    mFlickStickDrive = new FlickStickDrive(
      mSwerveDrive,
      () -> mDriverController.getLeftStickY(), 
      () -> mDriverController.getLeftStickX(),
      () -> mDriverController.getRightStickAngle(),
      () -> mDriverController.getRightStickPower()
    );

    // Config default commands
    mSwerveDrive.setDefaultCommand(mDefaultDrive); // Preference moment
    // mSwerveDrive.setDefaultCommand(mFlickStickDrive);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    mDriverController.a().whileTrue( // Rotate Swerve to zero radians
      new RotateSwerveToDrive(
        mSwerveDrive,
        () -> mDriverController.getLeftStickY(),
        () -> mDriverController.getLeftStickX(),
        () -> 0.0
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
