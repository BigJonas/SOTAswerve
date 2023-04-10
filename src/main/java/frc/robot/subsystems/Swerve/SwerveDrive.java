// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  protected final SwerveModule[] mModules;
  private final AHRS mGyro;
  private final SwerveDriveKinematics mKinematics;
  private final SwerveDrivePoseEstimator mOdometry;
  private final Timer mOdometryTimer;

  private final ProfiledPIDController mFwdTranslationController;
  private final ProfiledPIDController mStrTranslationController;
  private final ProfiledPIDController mRotTranslationController;

  private boolean mFieldCentricActive;
  private double mMaxWheelSpeed;
  private double mMaxAngularVelocity;
  private Translation2d mPointOfRotation;

  /** Creates a new Swerve. */
  public SwerveDrive(SwerveModule[] modules, AHRS gyro, SwerveDriveKinematics kinematics) {
    mModules = modules;
    mGyro = gyro;
    mKinematics = kinematics; 
    // TODO: Violates RAII dont know how to fix
    mOdometry = new SwerveDrivePoseEstimator(mKinematics, getRotation2d(), getModulePosition(), new Pose2d());  
    mOdometryTimer = new Timer();

    mFwdTranslationController = new ProfiledPIDController(0.0, 0.0, 0.0, null); // Should make a config class that contains this
    mStrTranslationController = new ProfiledPIDController(0.0, 0.0, 0.0, null);
    mRotTranslationController = new ProfiledPIDController(0.0, 0.0, 0.0, null);

    mPointOfRotation = new Translation2d(); // Default is 0.0
  }

  /**
   * Drives the swerve
   * @param fwd Forward velocity
   * @param str Strafe velocity
   * @param rot Angular velocity
   */
  public void drive(double fwd, double str, double rot) {
    fwd = MathUtil.clamp(fwd, -1, 1) * mMaxWheelSpeed;
    str = MathUtil.clamp(str, -1, 1) * mMaxWheelSpeed;
    rot = MathUtil.clamp(rot, -1, 1) * mMaxAngularVelocity;
    ChassisSpeeds speeds = new ChassisSpeeds(fwd, str, rot);
    if (mFieldCentricActive) speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation2d());
    drive(speeds);
  }

  /**
   * Drives the swerve 
   * @param speeds Desired chassis speeds
   */
  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(speeds, mPointOfRotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, mMaxWheelSpeed);
    drive(speeds);
  }

  /**
   * Drives the swerve
   * @param moduleStates Desired module states
   */
  public void drive(SwerveModuleState[] moduleStates) {
    for (int i = 0; i < moduleStates.length; i++) {
      mModules[i].setState(moduleStates[i]);
    }
  }

  /**
   * Sets field centric
   * @param fieldCentricActive Field centric active
   */
  public void setFieldCentric(boolean fieldCentricActive) {
    mFieldCentricActive = fieldCentricActive;
  }

  /**
   * Gets field centric
   * @return Whether field centric is active
   */
  public boolean getFieldCentric() {
    return mFieldCentricActive;
  }

  /**
   * Sets the point of rotation
   * @param pointOfRotation Desired point of rotation
   */
  public void setPointOfRotation(Translation2d pointOfRotation) {
    mPointOfRotation = pointOfRotation;
  }

  /**
   * Gets the measured module states of all the modules
   * @return Measured module states
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[mModules.length]; 
    for (int i = 0; i < mModules.length; i++) {
      moduleStates[i] = mModules[i].getState();
    }
    return moduleStates;
  }

  /**
   * Gets the measured module position of all the modules
   * @return Measured module position
   */
  public SwerveModulePosition[] getModulePosition() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[mModules.length];
    for (int i = 0; i < mModules.length; i++) {
      modulePositions[i] = mModules[i].getPosition();
    }
    return modulePositions;
  }

  /**
   * Updates odometry with gyro angle and module position
   */
  public void updateOdometry() {
    mOdometry.update(getRotation2d(), getModulePosition());
  }

  /**
   * Updates odometry with a precicted pose2d
   * @param precidctPose Predicted pose2d
   */
  public void updateOdometry(Pose2d precidctPose) {
    mOdometry.addVisionMeasurement(precidctPose, mOdometryTimer.get());
  }

  /**
   * Resets odometry
   * Note since using PoseEstimator then init pose is required
   * you can reset it with a blank pose to have it behave like regular odometry
   * IF YOU ARE USING ODOMETRY AS POSE ESTIMATOR CALL THIS ON START WITH AUTO INIT POSE
   * @param initPose Initial pose of robot
   */
  public void resetOdometry(Pose2d initPose) {
    mOdometry.resetPosition(getRotation2d(), getModulePosition(), initPose);
    mOdometryTimer.reset();
    mOdometryTimer.start();
  }

  /**
   * Gets the pose of the swerve according to odometry
   * @return Pose of the swerve (meters)
   */
  public Pose2d getPose2d() {
    return mOdometry.getEstimatedPosition();
  }

  /**
   * Gets rotation of the swerve 
   * @return Rotation of the swerve drive (yaw)
   */
  public Rotation2d getRotation2d() {
    return new Rotation2d(getYaw());
  }

  /**
   * Gets the rotations of the swerve
   * @return Rotations of the swerve drive (roll, pitch, yaw)
   */
  public Rotation3d getRotation3d() {
    return new Rotation3d(getRoll(), getPitch(), getYaw());
  }

  /**
   * Gets the yaw of the swerve 
   * @return Yaw of the swerve radians
   */
  public double getYaw() {
    return Math.toRadians(mGyro.getYaw());
  }

  /**
   * Gets the pitch of the swerve 
   * @return Pitch of the swerve radians
   */
  public double getPitch() {
    return Math.toRadians(mGyro.getPitch());
  }

  /**
   * Gets the roll of the swerve
   * @return Roll of the swerve radians
   */
  public double getRoll() {
    return Math.toRadians(mGyro.getRoll());
  }

  /**
   * Reset gyro
   */
  public void resetGyro() {
    mGyro.reset();
    mGyro.setAngleAdjustment(0.0);
  }

  /**
   * Sets the rotation of the Gyro
   * @param rotation Set angle of the gyro
   */
  public void setRotation(Rotation2d rotation) {
    resetGyro();
    mGyro.setAngleAdjustment(rotation.getDegrees());
  }

  /**
   * Gets the forward translation PID controller
   * @return Forward translation PID
   */
  public ProfiledPIDController getForwardTranslationPID() {
    return mFwdTranslationController;
  }

  /**
   * Gets the strafe translation PID controller
   * @return Strafe translation PID
   */
  public ProfiledPIDController getStrafeTranslationPID() {
    return mStrTranslationController;
  }

  /**
   * Gets the rotation translation PID controller
   * @return Rotation translation PID
   */
  public ProfiledPIDController getRotationTranslationController() {
    return mRotTranslationController;
  }

  /**
   * Disables the swerve
   */
  public void disable() {
    for (int i = 0; i < mModules.length; i++) {
      mModules[i].disable();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();


  }
}
