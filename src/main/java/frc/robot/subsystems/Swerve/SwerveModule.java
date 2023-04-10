// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Conversion;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.AnalogInput.*;

public class SwerveModule extends SubsystemBase {

  private final CANSparkMax mSpeedMotor;
  private final RelativeEncoder mSpeedEncoder;
  private final SparkMaxPIDController mSpeedPID; // Measurement in Meters Per Seconds
  private final SimpleMotorFeedforward mSpeedFF; // HAVE TO CHARACTERIZE IN MPS (GOOFY)

  private final CANSparkMax mAngleMotor;
  private final RelativeEncoder mAngleEncoder;
  private final AnalogInput mAnalogInput;
  private final SparkMaxPIDController mAnglePID; // Measurement in Radians

  private SwerveModuleState mState;
  private SwerveModuleState mLastState;

  private double mSpeedGearRatio;
  private double mLastSpeedGearRatio;

  private double kOffset;
  private double mMaxWheelSpeed;

  /** Creates a new SwerveModule. */
  public SwerveModule(CANSparkMax speedMotor, CANSparkMax angleMotor, AnalogInput analogInput) {
    mSpeedMotor = speedMotor;
    mSpeedEncoder = mSpeedMotor.getEncoder();
    mSpeedPID = mSpeedMotor.getPIDController();
    mSpeedFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0); // TODO: make constants and tune

    mAngleMotor = angleMotor;
    mAngleEncoder = mAngleMotor.getEncoder();
    mAnalogInput = analogInput;
    mAnglePID = mAngleMotor.getPIDController();

    mSpeedGearRatio = 0.0; // TODO: make constants

    configPIDControllers();
    configEncoders();
  }

  /**
   * Sets the state of the swerve module
   * @param desiredState Desired state of the swerve module
   */
  public void setState(SwerveModuleState desiredState) {
    mState = SwerveModuleState.optimize(desiredState, getAngle());
    // Dont change angle when speed is less than 1 percent
    if (Math.abs(mState.speedMetersPerSecond) <= mMaxWheelSpeed * 0.01) {
      mState.angle = mLastState.angle;
    }
  }

  /**
   * Gets the state of the swerve module 
   * @return Current measured state of the module
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getSpeed(),
      getAngle()
    );
  }

  /**
   * Gets the position of the module 
   * @return Current measured position of the module
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      getDistance(),
      getAngle()
    );
  }

  /**
   * Sets the speed gear ratio
   * @param gearRatio Desired speed gear ratio
   */
  public void setSpeedGearRatio(double gearRatio) {
    mSpeedGearRatio = gearRatio; 
  }

  /**
   * Get the speed of the module
   * @return Speed of the module MPS
   */
  public double getSpeed() {
    return mSpeedEncoder.getVelocity();
  }

  /**
   * Get the distance of the module
   * @return Distance of the module Meters
   */
  public double getDistance() {
    return mSpeedEncoder.getPosition();
  }

  /**
   * Get angle of the module
   * @return Angle of the module
   */
  public Rotation2d getAngle() {
    return new Rotation2d(mAngleEncoder.getPosition());
  }

  /**
   * Gets Analog input
   * @return Analog input for the module
   */
  public double getAnalogInput() {
    return -1 * MathUtil.inputModulus(mAnalogInput.getAverageVoltage() - kOffset, 0.0, COUNTS_PER_REVOLUTION) + COUNTS_PER_REVOLUTION;
  }

  /**
   * Config PID controllers to default state
   */
  private void configPIDControllers() {
    mSpeedPID.setP(0.0);
    mSpeedPID.setI(0.0);
    mSpeedPID.setD(0.0);
    mSpeedPID.setFF(0.0); // This is essentially KS in simple motor FF up to you to use it 

    mAnglePID.setP(0.0);
    mAnglePID.setI(0.0);
    mAnglePID.setD(0.0);
    mAnglePID.setFF(0.0); // This is essentially KS in simple motor FF up to you to use it 
    
    mAnglePID.setPositionPIDWrappingEnabled(true);
    mAnglePID.setPositionPIDWrappingMinInput(0.0);
    mAnglePID.setPositionPIDWrappingMaxInput(2 * Math.PI);

  }

  /**
   * Config encoders to default state
   */
  private void configEncoders() {
    updateSpeedEncoder();
    mSpeedEncoder.setPosition(0.0);

    mAngleEncoder.setPositionConversionFactor(0.0); // TODO: Convert to Radians
    mAngleEncoder.setVelocityConversionFactor(0.0); // TODO: Convert to Radians Per Second
    updateAngleEncoder();
  }

  /**
   * Updates the encoder on the SparkMax with the current gear ratio
   * Our team is silly and likes to change speed gear ratios 
   */
  private void updateSpeedEncoder() { 
    double rotationToMeters = WHEEL_CIRCUMFERENCE / mSpeedGearRatio; // TODO: maybe extract this to something else 
    double rpmPerMetersToSecond = rotationToMeters / 60.0;
    mSpeedEncoder.setPositionConversionFactor(rotationToMeters);
    mSpeedEncoder.setVelocityConversionFactor(rpmPerMetersToSecond);
  }

  /**
   * Updates the encoder on the SparkMax with the value on the AnalogEncoder
   */
  private void updateAngleEncoder() {
    mAngleEncoder.setPosition(Conversion.analogInputToRadians(getAnalogInput()));
  }

  /**
   * Disables the module 
   */
  public void disable() {
    mState.speedMetersPerSecond = 0.0;
  }

  /**
   * Motor pids are set in periodic so MAKE SURE to disable
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (mLastSpeedGearRatio != mSpeedGearRatio) {
      updateSpeedEncoder();
    }
    updateAngleEncoder(); // TODO: Dont really want this running every 20ms for usage 

    // Currently not using PID Slot would like to in edge cases 
    mSpeedPID.setReference(mState.speedMetersPerSecond, ControlType.kVelocity, 0, mSpeedFF.calculate(mState.speedMetersPerSecond));
    mAnglePID.setReference(mState.angle.getRadians(), ControlType.kPosition, 0);

    mLastSpeedGearRatio = mSpeedGearRatio;
    mLastState = mState;
  }
}