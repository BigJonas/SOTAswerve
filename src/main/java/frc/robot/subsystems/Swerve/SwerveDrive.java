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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LazySwerveModuleStateFactory;

/**
 * Basic swerve drive 
 * NOTE:
 * MUST DECLARE MODULES AND KINEMATICS WITH THE ORDER
 * FRONT LEFT, FRONT RIGHT, BACK LEFT, BACK RIGHT
 * If you plan on using the Pose Estimator feature with vision
 * You must call {@link SwerveDrive#resetOdometry() resetOdometry}
 * on start with an auto initial pose
 */
public class SwerveDrive extends SubsystemBase {
  protected final SwerveModule[] mModules;
  private final AHRS mGyro;
  private final SwerveDriveKinematics mKinematics;
  private final LazySwerveModuleStateFactory mModuleStateFactory;
  private final SwerveDrivePoseEstimator mOdometry;
  private final Timer mOdometryTimer;

  private final ProfiledPIDController mFwdTranslationController;
  private final ProfiledPIDController mStrTranslationController;
  private final ProfiledPIDController mRotTranslationController;

  private final Field2d mField;

  private boolean mFieldCentricActive;
  private Translation2d mPointOfRotation;

  protected double mSpeedGearRatio;
  private double mMaxWheelSpeed;
  private double mMaxAngularVelocity;

  /**
   * A basic swerve drive 
   * @param modules Swerve modules (order Front Left, Front Right, Back Left, Back Right)
   * @param gyro Gyro
   * @param kinematics Swerve kinematics (order Front Left, Front Right, Back Left, Back Right)
   */
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

    mField = new Field2d();

    mPointOfRotation = new Translation2d(); // Default is 0.0

    mSpeedGearRatio = 0.0; //TODO: Constants please
    mMaxWheelSpeed = getMaxWheelSpeed();
    mMaxAngularVelocity = getMaxAngularVelocity();

    mModuleStateFactory = new LazySwerveModuleStateFactory(kinematics, getModuleStates(), mMaxWheelSpeed);

  }

  /**
   * Drives the swerve
   * @param fwd Forward velocity
   * @param str Strafe velocity
   * @param rot Angular velocity
   */
  public void drive(double fwd, double str, double rot) {
    // Clamps and scales inputs
    fwd = MathUtil.clamp(fwd, -1, 1) * mMaxWheelSpeed;
    str = MathUtil.clamp(str, -1, 1) * mMaxWheelSpeed;
    rot = MathUtil.clamp(rot, -1, 1) * mMaxAngularVelocity;

    // Converts inputs into robot relative chassis speeds
    ChassisSpeeds speeds = new ChassisSpeeds(fwd, str, rot); 
    // If field centric is active convert robot relative chassis speeds into field relative speeds
    if (mFieldCentricActive) speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation2d());
    drive(speeds);
  }

  /**
   * Drives the swerve 
   * @param speeds Desired chassis speeds
   */
  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = mModuleStateFactory.generateModuleStates(speeds, mPointOfRotation);
    // TODO: desaturate module states off of whatever that one thing jons talkin about
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
   * Gets theoretical max wheel speed of the drivetrain
   * @return Max wheel speed of the drivetrain
   */
  private double getMaxWheelSpeed() {
    return SwerveUtil.getNeoMaxWheelSpeed(mSpeedGearRatio);
  }

  /**
   * Gets theoretical max angular velocity of the drivetrain
   * @return Max angular velocity of the drivetrain
   */
  private double getMaxAngularVelocity() {
    return SwerveUtil.getMaxAngularVelocity(getMaxWheelSpeed());
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
    mField.setRobotPose(initPose);
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
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Swerve Drive");

    builder.addStringProperty("Current Command", super.getCurrentCommand()::getName, null);
    builder.addBooleanProperty("Field Centric Active", this::getFieldCentric, null);
    builder.addDoubleProperty("Gyro Angle (radians)", this::getYaw, null);
    
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    mField.setRobotPose(getPose2d());

  }
}
