// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * Shifting swerve drive
 * Retains functionality of swerve class but with
 * the ability to shift between two speed gear ratios
 * See {@link SwerveDrive} to find important drive code
 */
public class ShiftingSwerveDrive extends SwerveDrive {
  private final DoubleSolenoid mShifter;

  private double[] kGearRatios = new double[2];
  /** Creates a new ShiftingSwerveDrive. */
  public ShiftingSwerveDrive(SwerveModule[] modules, DoubleSolenoid shifter, AHRS gyro, SwerveDriveKinematics kinematics) {
    super(modules, gyro, kinematics);

    mShifter = shifter;

    kGearRatios[0] = 0.0; // TODO: make constants file
    kGearRatios[1] = 0.0;
  }

  /**
   * Changes the gear of the drive train 
   * @param gear Desired gear of the drive train 0 is low 1 is high
   */
  public void shift(int gear) {
    mShifter.set(gear == 1 ? Value.kForward : Value.kReverse); // TODO: Maybe make constants for this idk
    for (int i = 0; i < super.mModules.length; i++) {
      super.mModules[i].setSpeedGearRatio(kGearRatios[i]);
    } 
  }

  /**
   * Gets the gear of the drive train
   * @return Gear of the drive train 0 is low 1 is high
   */
  public int getGear() {
    return mShifter.get() == Value.kForward ? 1 : 0;
  }

  
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("Current Gear", () -> getGear() == 1 ? true : false , null);
  } 

}
