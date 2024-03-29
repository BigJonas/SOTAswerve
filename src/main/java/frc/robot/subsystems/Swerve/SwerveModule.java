// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.AnalogInput.*;

/**
 * Swerve Module using Neos for Drive and Angling
 */
public class SwerveModule extends SubsystemBase {

  /**
   * 0 Front left
   * 1 Front right
   * 2 Back left
   * 3 Back right
   */
  private final int mModuleNum; 

  private final CANSparkMax mSpeedMotor;
  private final RelativeEncoder mSpeedEncoder;

  private final SparkMaxPIDController mSpeedPID; // Measurement in Meters Per Seconds
  private final SimpleMotorFeedforward mSpeedFF; // HAVE TO CHARACTERIZE IN MPS (GOOFY)

  private final CANSparkMax mAngleMotor;
  private final RelativeEncoder mAngleEncoder;
  private final AnalogInput mAnalogInput;

  private final SparkMaxPIDController mAnglePID; // Measurement in Radians
  private final ProfiledPIDController mAngleTrapezoidProfile; // Used only for trapezoid profile feature
  private final SimpleMotorFeedforward mAngleFF; // Characterize in RPS (still goofy)

  private double mSpeedGearRatio;
  private double mLastSpeedGearRatio;

  private double mAngleGearRatio; // Doesnt need mLastAngleGearRatio however who knows

  private double kOffset;

  /** Creates a new SwerveModule. */
  public SwerveModule(CANSparkMax speedMotor, CANSparkMax angleMotor, AnalogInput analogInput) {
    mModuleNum = 0; // TODO: make constants

    mSpeedMotor = speedMotor;
    mSpeedEncoder = mSpeedMotor.getEncoder();

    mSpeedPID = mSpeedMotor.getPIDController();
    mSpeedFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0); // TODO: make constants and tune

    mAngleMotor = angleMotor;
    mAngleEncoder = mAngleMotor.getEncoder();
    mAnalogInput = analogInput;

    mAnglePID = mAngleMotor.getPIDController();
    mAngleTrapezoidProfile = new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(0.0, 0.0)); // TODO: Constants blah blah
    mAngleTrapezoidProfile.enableContinuousInput(0.0, 2 * Math.PI);
    mAngleFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0); // TODO: guess whhat make constants

    mSpeedGearRatio = 0.0; // TODO: make constants
    mLastSpeedGearRatio = mSpeedGearRatio;
    mAngleGearRatio = 0.0; // TODO: make constants
    kOffset = 0.0; //TODO: make constants

    configPIDControllers();
    configEncoders();
  }

  /**
   * Sets the state of the swerve module
   * @param desiredState Desired state of the swerve module
   */
  public void setState(SwerveModuleState desiredState) {
    desiredState = SwerveModuleState.optimize(desiredState, getRotation());
    setSpeed(desiredState.speedMetersPerSecond);
    setAngle(desiredState.angle.getRadians());
  }

  /**
   * Sets the voltage of the Speed motor and forces it straight
   * @param voltage Voltage of the motor
   */
  public void setCharacterizationSpeedVoltage(double voltage) {
    mSpeedMotor.setVoltage(voltage);
    setAngle(0.0);
  }

  /**
   * Sets the voltage of the Angle motor
   * @param voltage Voltage of the motor
   */
  public void setCharacterizationAngleVoltage(double voltage) {
    mAngleMotor.setVoltage(voltage);
  }

  /**
   * Sets the speed of the module meters per second
   * @param metersPerSecond
   */
  public void setSpeed(double metersPerSecond) {
    // Currently not using PID Slot would like to in edge cases 
    mSpeedPID.setReference(
      metersPerSecond,
      ControlType.kVelocity,
      0, 
      mSpeedFF.calculate(metersPerSecond),
      ArbFFUnits.kVoltage
    );
  }

  /**
   * Sets the state of the module with 
   * @param positionProfile Position profile
   */
  public void setAngle(double radians) {
    mAngleTrapezoidProfile.calculate(getAngle(), radians);
    TrapezoidProfile.State state = mAngleTrapezoidProfile.getSetpoint();
    radians = state.position; // Sets radians setpoint to follow 
    double radiansPerSecond = state.velocity;
    mAnglePID.setReference(
      radians, 
      ControlType.kPosition, 
      0,
      mAngleFF.calculate(radiansPerSecond),
      ArbFFUnits.kVoltage
    );
  }

  /**
   * Gets the state of the swerve module 
   * @return Current measured state of the module
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getSpeed(),
      getRotation()
    );
  }

  /**
   * Gets the position of the module 
   * @return Current measured position of the module
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      getDistance(),
      getRotation()
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
   * @return Angle of the module (Radians)
   */
  public double getAngle() {
    return mAngleEncoder.getPosition();
  }

  /**
   * Gets the rotation of the module
   * @return Rotation of the module
   */
  public Rotation2d getRotation() {
    return new Rotation2d(getAngle());
  }

  /**
   * Gets the angle velocity of the module
   * @return Angle velocity of the module
   */
  public double getAngleVelocity() {
    return mAngleEncoder.getVelocity();
  }

  /**
   * Gets Analog input
   * @return Analog input for the module
   */
  public double getAnalogInput() {
    return -1 * MathUtil.inputModulus(getAnalogInputNoOffset() - kOffset, 0.0, COUNTS_PER_REVOLUTION) + COUNTS_PER_REVOLUTION;
  }

  /**
   * Gets the analog input without offset
   * @return Analog input without offset
   */
  public double getAnalogInputNoOffset() {
    return mAnalogInput.getAverageVoltage();
  }

  /**
   * Config PID controllers to default state
   */
  private void configPIDControllers() {
    mSpeedPID.setP(0.0);
    mSpeedPID.setI(0.0);
    mSpeedPID.setD(0.0);
    mSpeedPID.setFF(0.0); // This is essentially KS in simple motor FF up to you to use it 

    mSpeedPID.setIZone(0.0);
    mSpeedPID.setIMaxAccum(0.0, 0); // Why does this one have mandatory maxAccum

    mAnglePID.setP(0.0);
    mAnglePID.setI(0.0);
    mAnglePID.setD(0.0);
    mAnglePID.setFF(0.0); // This is essentially KS in simple motor FF up to you to use it 
    
    mSpeedPID.setIZone(0.0);
    mSpeedPID.setIMaxAccum(0.0, 0); // Why does this one have mandatory maxAccum
    
    mAnglePID.setPositionPIDWrappingEnabled(true);
    mAnglePID.setPositionPIDWrappingMinInput(0.0);
    mAnglePID.setPositionPIDWrappingMaxInput(2 * Math.PI);

  }

  /**
   * Config encoders to default state
   */
  private void configEncoders() {
    updateSpeedConversionFactor();
    resetSpeedEncoder();

    updateAngleConversionFactor();
    resetAngleEncoder();
  }

  /**
   * Updates the conversion factor on the SparkMax with the current speed gear ratio
   * Our team is silly and likes to change speed gear ratios   
   */
  private void updateSpeedConversionFactor() {
    double rotationToMeters = SwerveUtil.getNeoRotationToMeters(mSpeedGearRatio); 
    double rpmToMetersPerSecond = rotationToMeters / 60.0;
    mSpeedEncoder.setPositionConversionFactor(rotationToMeters);
    mSpeedEncoder.setVelocityConversionFactor(rpmToMetersPerSecond);
  }

  /**
   * Reset speed encoder
   */
  private void resetSpeedEncoder() {
    mSpeedEncoder.setPosition(0.0);
  }

  /**
   * Updates the conversion factor on the SparkMax with the current angle gear ratio
   * Dont think this is all too useful but it follows what is done with the speed so :)
   */
  private void updateAngleConversionFactor() {
    double rotationToRadians = SwerveUtil.getNeoRotationToMeters(mAngleGearRatio); 
    double rpmToRadiansPerSecond = rotationToRadians / 60.0;
    mAngleEncoder.setPositionConversionFactor(rotationToRadians);
    mAngleEncoder.setVelocityConversionFactor(rpmToRadiansPerSecond);
  }

  /**
   * Updates the encoder on the SparkMax with the value on the AnalogEncoder
   */
  private void resetAngleEncoder() {
    mAngleEncoder.setPosition(SwerveUtil.analogInputToRadians(getAnalogInput()));
  }

  /**
   * Disables the module 
   */
  public void disable() {
    mSpeedMotor.set(0.0);
    mAngleMotor.set(0.0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Swerve Module " + mModuleNum);

    builder.addDoubleProperty("Angle (Degrees)", getRotation()::getDegrees, null);
    builder.addDoubleProperty("Meters Per Second", mSpeedEncoder::getVelocity, null);
    builder.addDoubleProperty("No Offset", this::getAnalogInputNoOffset, null);
    builder.addDoubleProperty("Current Speed Gear Ratio", () -> mSpeedGearRatio, null);
  }

  /**
   * Motor pids are set in periodic so MAKE SURE to disable
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (mLastSpeedGearRatio != mSpeedGearRatio) {
      updateSpeedConversionFactor();
    }
    // If the drift between the Spark encoder and the absolute encoder is greater than 1 percent then reset 
    if (Math.abs(getAngle() - SwerveUtil.analogInputToRadians(getAnalogInput())) < ((2 * Math.PI) * 0.01)) {
      resetAngleEncoder(); 
    }

    mLastSpeedGearRatio = mSpeedGearRatio;
  }

}
