// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Swerve.*;

/** Add your docs here. */
public class Conversion {

    /**
     * Converts inches to meters
     * @param inches inches
     * @return meters
     */
    public static double inchesToMeters(double inches) {
        return inches * 0.0254; // Thanks hayden
    }
    
    /**
     * Converts 5.0v analog input encoder to radians
     * @param analog 5.0v analog input 
     * @return Radians of the encoder
     */
    public static double analogInputToRadians(double analog) {
        return analog * (2 * Math.PI) / AnalogInput.COUNTS_PER_REVOLUTION;
    }

    /**
     * Gets the Rotation to Meters of a Neo motor
     * @param gearRatio Current gear ratio of the mechanism
     * @return Rotatio to Meters
     */
    public static double getNeoRotationToMeters(double gearRatio) {
        return WHEEL_CIRCUMFERENCE / gearRatio / Neo.COUNTS_PER_REVOLUTION; // TODO: Double check if the division by CPR is needed
    }

    /**
     * Gets the Rotation to Radians of a Neo motor
     * @param gearRatio Current gear ratio of the mechanism
     * @return Rotation to Radians
     */
    public static double getNeoRotationToRadians(double gearRatio) {
        return (Math.PI * 2) / gearRatio / Neo.COUNTS_PER_REVOLUTION;
    }

}
