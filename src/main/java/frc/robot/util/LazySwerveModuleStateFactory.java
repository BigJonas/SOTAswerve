// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class LazySwerveModuleStateFactory {

    private SwerveDriveKinematics mKinematics;
    private SwerveModuleState[] mLastStates;
    private double mMaxWheelSpeed;

    public LazySwerveModuleStateFactory(SwerveDriveKinematics kinematics, SwerveModuleState[] currentStates, double maxWheelSpeed) {
        mKinematics = kinematics;
        mLastStates = currentStates;
        mMaxWheelSpeed = maxWheelSpeed;
    }

    public LazySwerveModuleState[] generateModuleStates(ChassisSpeeds mSpeeds, Translation2d pointOfRotation) {
        SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(mSpeeds, pointOfRotation);
        LazySwerveModuleState[] lazyStates = new LazySwerveModuleState[mLastStates.length];
        for (int i = 0; i < moduleStates.length; i++) {
            lazyStates[i] = new LazySwerveModuleState(moduleStates[i], mLastStates[i], mMaxWheelSpeed);
        }
        mLastStates = lazyStates;
        return lazyStates;
    }

    public void setMaxWheelSpeed(double maxWheelSpeed) {
        mMaxWheelSpeed = maxWheelSpeed;
    }
}
