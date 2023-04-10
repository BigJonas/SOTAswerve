// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

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
        return 0.0;
    }
}
