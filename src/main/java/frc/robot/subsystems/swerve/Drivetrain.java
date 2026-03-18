// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.DrivebaseConstants.ModuleConstants;
import frc.robot.Constants.DrivebaseConstants.ModuleConstants.BackLeftModule;
import frc.robot.Constants.DrivebaseConstants.ModuleConstants.BackRightModule;
import frc.robot.Constants.DrivebaseConstants.ModuleConstants.FrontLeftModule;
import frc.robot.Constants.DrivebaseConstants.ModuleConstants.FrontRightModule;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  public static final Pigeon2 pigeon = new Pigeon2(1);

  private final SwerveModule m_frontLeft =
      new SwerveModule(
          FrontLeftModule.FRONT_LEFT_DRIVE_MOTOR_ID,
          FrontLeftModule.FRONT_LEFT_AZIMUTH_MOTOR_ID,
          FrontLeftModule.FRONT_LEFT_ENCODER_PORT,
          ModuleConstants.ENCODER_TICKS_PER_ROTATION,
          FrontLeftModule.FRONT_LEFT_ENCODER_OFFSET,
          "Front Left",
          FrontLeftModule.FRONT_LEFT_DRIVE_MOTOR_INVERTED,
          FrontLeftModule.FRONT_LEFT_AZIMUTH_MOTOR_INVERTED);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          FrontRightModule.FRONT_RIGHT_DRIVE_MOTOR_ID,
          FrontRightModule.FRONT_RIGHT_AZIMUTH_MOTOR_ID,
          FrontRightModule.FRONT_RIGHT_ENCODER_PORT,
          ModuleConstants.ENCODER_TICKS_PER_ROTATION,
          FrontRightModule.FRONT_RIGHT_ENCODER_OFFSET,
          "Front Right",
          FrontRightModule.FRONT_RIGHT_DRIVE_MOTOR_INVERTED,
          FrontRightModule.FRONT_RIGHT_AZIMUTH_MOTOR_INVERTED);

  private final SwerveModule m_backLeft =
      new SwerveModule(
          BackLeftModule.BACK_LEFT_DRIVE_MOTOR_ID,
          BackLeftModule.BACK_LEFT_AZIMUTH_MOTOR_ID,
          BackLeftModule.BACK_LEFT_ENCODER_PORT,
          ModuleConstants.ENCODER_TICKS_PER_ROTATION,
          BackLeftModule.BACK_LEFT_ENCODER_OFFSET,
          "Back Left",
          BackLeftModule.BACK_LEFT_DRIVE_MOTOR_INVERTED,
          BackLeftModule.BACK_LEFT_AZIMUTH_MOTOR_INVERTED);

  private final SwerveModule m_backRight =
      new SwerveModule(
          BackRightModule.BACK_RIGHT_DRIVE_MOTOR_ID,
          BackRightModule.BACK_RIGHT_AZIMUTH_MOTOR_ID,
          BackRightModule.BACK_RIGHT_ENCODER_PORT,
          ModuleConstants.ENCODER_TICKS_PER_ROTATION,
          BackRightModule.BACK_RIGHT_ENCODER_OFFSET,
          "Back Right",
          BackRightModule.BACK_RIGHT_DRIVE_MOTOR_INVERTED,
          BackRightModule.BACK_RIGHT_AZIMUTH_MOTOR_INVERTED);

  private final Supplier<Rotation2d> m_gyroSupplier;

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          DrivebaseConstants.FRONT_LEFT_LOCATION, DrivebaseConstants.FRONT_RIGHT_LOCATION,
          DrivebaseConstants.BACK_LEFT_LOCATION, DrivebaseConstants.BACK_RIGHT_LOCATION);

  private final SwerveDriveOdometry m_odometry;

  private final SwerveModulePosition[] m_lastPos;

  private final SwerveDrivePoseEstimator poseEstimator;

  private RobotConfig config =
      new RobotConfig(
          Constants.ROBOT_MASS,
          Constants.ROBOT_MOI,
          new ModuleConfig(
              DrivebaseConstants.WHEEL_DIAMETER_INCHES,
              DrivebaseConstants.TOP_SPEED_METERS_PER_SEC,
              DrivebaseConstants.WHEEL_FRICTION_COEFFICIENT,
              DCMotor.getKrakenX60(1).withReduction(DrivebaseConstants.DRIVE_GEAR_RATIO),
              DrivebaseConstants.MAX_CURRENT_AMPS,
              1),
          getModuleTranslations());

  /**
   * Constructs the Drivetrain with a supplier that returns the current robot heading.
   *
   * @param gyroSupplier A lambda or method reference that returns current heading as Rotation2d
   * @param initialPose The initial pose of the robot
   */
  public Drivetrain(Supplier<Rotation2d> gyroSupplier, Pose2d initialPose) {
    this.m_gyroSupplier = gyroSupplier;

    m_lastPos =
        new SwerveModulePosition[] {
          m_frontLeft.getSwervePosition(),
          m_frontRight.getSwervePosition(),
          m_backLeft.getSwervePosition(),
          m_backRight.getSwervePosition()
        };

    m_odometry =
        new SwerveDriveOdometry(m_kinematics, m_gyroSupplier.get(), m_lastPos, initialPose);

    poseEstimator =
        new SwerveDrivePoseEstimator(m_kinematics, m_gyroSupplier.get(), m_lastPos, getPose());

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        // Robot pose supplier
        this::getPose,
        // Method to reset odometry (will be called if your auto has a starting pose)
        this::resetOdometry,
        // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::getChassisSpeeds,
        // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
        // Also optionally outputs individual module feedforwards
        (speeds, feedforwards) -> driveVelocity(speeds),
        // PPHolonomicController is the built in path following controller for holonomic drive
        // trains
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
        // The robot configuration
        config,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        // Reference to this subsystem to set requirements
        this);
  }

  /**
   * Drives the robot using desired speeds and rotation.
   *
   * @param xSpeed Speed in the X direction (forward).
   * @param ySpeed Speed in the Y direction (sideways).
   * @param rot Angular speed (radians/sec).
   * @param fieldRelative Whether movement should be relative to the field.
   * @param periodSeconds The loop period (dt).
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    ChassisSpeeds speeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pigeon.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot);

    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, periodSeconds));

    // Prevent over-speed by scaling wheel speeds to the configured maximum
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DrivebaseConstants.TOP_SPEED_METERS_PER_SEC);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    // Horrible hack to get the modules to stop spinning if no drive input.
    if ((xSpeed == 0) && (ySpeed == 0) && (rot == 0)) {
      stopModules();
    }
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        pigeon.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getSwervePosition(),
          m_frontRight.getSwervePosition(),
          m_backLeft.getSwervePosition(),
          m_backRight.getSwervePosition()
        });
  }

  /** Gets the current robot pose */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /** Resets the odometry to a specific pose */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        pigeon.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getSwervePosition(),
          m_frontRight.getSwervePosition(),
          m_backLeft.getSwervePosition(),
          m_backRight.getSwervePosition()
        },
        pose);
  }

  /** Stops all swerve modules */
  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  /** Sets all modules to X formation for defense */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /** Gets current chassis speeds */
  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(
        m_frontLeft.getSwerveState(),
        m_frontRight.getSwerveState(),
        m_backLeft.getSwerveState(),
        m_backRight.getSwerveState());
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      DrivebaseConstants.FRONT_LEFT_LOCATION, DrivebaseConstants.FRONT_RIGHT_LOCATION,
      DrivebaseConstants.BACK_LEFT_LOCATION, DrivebaseConstants.BACK_RIGHT_LOCATION
    };
  }

  /** Should be called periodically to update odometry and SmartDashboard */
  public void periodic() {
    // Update odometry
    updateOdometry();

    // Update module calibration info on SmartDashboard (no parameters needed)
    m_frontLeft.updateSmartDashboard();
    m_frontRight.updateSmartDashboard();
    m_backLeft.updateSmartDashboard();
    m_backRight.updateSmartDashboard();

    // Robot pose information
    Pose2d currentPose = getPose();
    Logger.recordOutput("Robot X (m)", currentPose.getX());
    Logger.recordOutput("Robot Y (m)", currentPose.getY());
    Logger.recordOutput("Robot Rotation (deg)", currentPose.getRotation().getDegrees());

    // Gyro information
    Logger.recordOutput("Gyro Angle (deg)", m_gyroSupplier.get().getDegrees());

    // Current chassis speeds
    ChassisSpeeds speeds = getChassisSpeeds();
    Logger.recordOutput("Chassis X Speed (m/s)", speeds.vxMetersPerSecond);
    Logger.recordOutput("Chassis Y Speed (m/s)", speeds.vyMetersPerSecond);
    Logger.recordOutput("Chassis Angular Speed (rad/s)", speeds.omegaRadiansPerSecond);

    // Module states for debugging
    Logger.recordOutput("FL Speed (m/s)", m_frontLeft.getSwerveState().speedMetersPerSecond);
    Logger.recordOutput("FL Angle (deg)", m_frontLeft.getSwerveState().angle.getDegrees());

    Logger.recordOutput("FL Speed (m/s)", m_frontLeft.getSwerveState().speedMetersPerSecond);
    Logger.recordOutput("FL Angle (deg)", m_frontLeft.getSwerveState().angle.getDegrees());

    Logger.recordOutput("BL Speed (m/s)", m_backLeft.getSwerveState().speedMetersPerSecond);
    Logger.recordOutput("BL Angle (deg)", m_backLeft.getSwerveState().angle.getDegrees());

    Logger.recordOutput("BR Speed (m/s)", m_backRight.getSwerveState().speedMetersPerSecond);
    Logger.recordOutput("BR Angle (deg)", m_backRight.getSwerveState().angle.getDegrees());

    // Control buttons
    handleSmartDashboardButtons();
  }

  /** Handle SmartDashboard button inputs */
  private void handleSmartDashboardButtons() {
    // Initialize buttons if they don't exist
    if (!SmartDashboard.containsKey("Reset Odometry")) {
      SmartDashboard.putBoolean("Reset Odometry", false);
    }
    if (!SmartDashboard.containsKey("Set X Formation")) {
      SmartDashboard.putBoolean("Set X Formation", false);
    }
    if (!SmartDashboard.containsKey("Stop All Modules")) {
      SmartDashboard.putBoolean("Stop All Modules", false);
    }

    // Reset odometry button
    if (SmartDashboard.getBoolean("Reset Odometry", false)) {
      resetOdometry(new Pose2d());
      SmartDashboard.putBoolean("Reset Odometry", false);
      System.out.println("Odometry reset to origin");
    }

    // X formation button
    if (SmartDashboard.getBoolean("Set X Formation", false)) {
      setX();
      SmartDashboard.putBoolean("Set X Formation", false);
      System.out.println("Set to X formation");
    }

    // Stop all modules button
    if (SmartDashboard.getBoolean("Stop All Modules", false)) {
      stopModules();
      SmartDashboard.putBoolean("Stop All Modules", false);
      System.out.println("All modules stopped");
    }
  }

  /**
   * Adds a vision measurement to update the robot's pose
   *
   * @param visionMeasurement Adds a measurement from a vision coprocessor
   * @param timestamp The timestamp of the vision measurement in seconds
   * @param stdDevs Standard deviations of the pose estimate (x position in meters, y position in
   *     meters, and heading in radians).
   */
  public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestamp, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestamp, stdDevs);
  }

  public void driveVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = m_kinematics.toSwerveModuleStates(discreteSpeeds);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    m_frontLeft.setDesiredState(setpointStates[0]);
    m_frontRight.setDesiredState(setpointStates[1]);
    m_backLeft.setDesiredState(setpointStates[2]);
    m_backRight.setDesiredState(setpointStates[3]);

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }
}
