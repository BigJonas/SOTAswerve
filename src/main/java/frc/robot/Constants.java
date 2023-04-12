// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.Conversion;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class AnalogInput {
    public static final double COUNTS_PER_REVOLUTION = 5.0;
  }

  public static class Swerve {
    public static final double TRACK_WIDTH = 0.0;
    public static final double WHEEL_BASE = 0.0;

    public static final double WHEEL_DIAMETER = Conversion.inchesToMeters(4.0);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  }

  public static class Neo {
    public static final double COUNTS_PER_REVOLUTION = 48.0;
    public static final double FREE_SPEED = 5820;
  }
}
