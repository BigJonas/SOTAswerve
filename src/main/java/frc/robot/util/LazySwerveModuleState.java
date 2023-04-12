// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * A swerve module state that doesnt change the angle of the module based off the max speed
 * If the desired speed is less than 1 percent of the max speed dont angle
 */
public class LazySwerveModuleState extends SwerveModuleState {
    public LazySwerveModuleState(SwerveModuleState desiredState, SwerveModuleState lastState, double maxWheelSpeed) {
        super(desiredState.speedMetersPerSecond, desiredState.angle);
        // If speed is less than 1 percent of speed then dont change angle
        if(Math.abs(speedMetersPerSecond) < maxWheelSpeed * 0.01) { 
            angle = lastState.angle;
        }
    }
}
