// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.ExternalEncoder;
import com.thethriftybot.devices.ThriftyNova.MotorType;
import com.thethriftybot.devices.ThriftyNova.PIDSlot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.DrivebaseConstants.ModuleConstants;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  // Motors
  private final TalonFX m_driveMotor;
  private final ThriftyNova m_azimuthMotor;

  // Encoder (only used for Thrifty absolute encoder)
  private final AnalogEncoder m_thriftyEncoder;

  // Encoder configuration
  private final double m_encoderTicksPerRevolution;
  // Used only for Thrifty encoder (RoboRIO reading), Nova just stores for persistence
  private double m_encoderOffsetTicks;
  private final String m_moduleName;

  // Per-module inversion configuration
  private final boolean m_driveInverted;
  private final boolean m_azimuthInverted;

  // PID controller for Thrifty encoder (RIO-side control)
  private final PIDController m_turningPID = new PIDController(0.07, 0.0, 0.0);
  private Rotation2d m_desiredAngle = new Rotation2d();

  // (removed unused m_hasCheckedSavedOffset flag)

  // Drive motor conversion factors

  private static final int NEO_ENCODER_TICKS_PER_REV = 42;

  private static final double DRIVE_TICKS_TO_METERS =
      (DrivebaseConstants.WHEEL_DIAMETER_METERS * Math.PI)
          / (NEO_ENCODER_TICKS_PER_REV * DrivebaseConstants.DRIVE_GEAR_RATIO);

  public SwerveModule(
      int driveMotorId,
      int azimuthMotorId,
      int encoderPort,
      double encoderTicksPerRevolution,
      double encoderOffsetTicks,
      String moduleName,
      boolean driveInverted,
      boolean azimuthInverted) {

    m_moduleName = moduleName;
    m_encoderTicksPerRevolution = encoderTicksPerRevolution;
    m_driveInverted = driveInverted;
    m_azimuthInverted = azimuthInverted;

    // Initialize motors
    m_driveMotor = new TalonFX(driveMotorId, CANBus.roboRIO());
    m_azimuthMotor = new ThriftyNova(azimuthMotorId, MotorType.MINION);
    // Set full range and expected zero so ticks map 1:1 with the configured encoder
    m_thriftyEncoder = new AnalogEncoder(encoderPort, m_encoderTicksPerRevolution, 0.0);

    configureDriveMotor();
    configureAzimuthMotor();
    initializeOffset(encoderOffsetTicks);

    // Configure turning PID for continuous input (-180 to 180 degrees)
    m_turningPID.enableContinuousInput(-Math.PI, Math.PI);

    System.out.println(m_moduleName + " module initialized successfully");
  }

  /** Configure the drive motor with PID and feedforward */
  private void configureDriveMotor() {
    // m_driveMotor.factoryReset();

    // Configure drive PID and feedforward
    // PID tuned for rotation units instead of ticks (scaled up by NEO_ENCODER_TICKS_PER_REV = 42)
    // m_driveMotor.pid0.setPID(new PIDController(0.0042, 0.0, 0.0));
    // Set feedforward based on mechanism characteristics:
    // FF = 1.0 / maxRevPerSec (for velocity control in rotations/sec)
    double estimatedMaxMps = DrivebaseConstants.TOP_SPEED_METERS_PER_SEC;
    double maxRevPerSec =
        estimatedMaxMps
            / (DrivebaseConstants.WHEEL_DIAMETER_METERS
                * Math.PI
                / DrivebaseConstants.DRIVE_GEAR_RATIO);
    // m_driveMotor.pid0.setFF(1.0 / maxRevPerSec);
    // m_driveMotor.usePIDSlot(PIDSlot.SLOT0);
    // m_driveMotor.setInversion(m_driveInverted);

    // Set current limits and voltage compensation for motor safety and performance
    TalonFXConfigurator talonFXConfigurator = m_driveMotor.getConfigurator();
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.SupplyCurrentLimit = DrivebaseConstants.MAX_CURRENT_AMPS;
    talonFXConfigurator.apply(limitConfigs);
  }

  /** Configure the azimuth motor based on encoder type */
  private void configureAzimuthMotor() {

    switch (ModuleConstants.ENCODER_SELECTED) {
      case REDUX_ENCODER:
        m_azimuthMotor.setExternalEncoder(ExternalEncoder.REDUX_ENCODER);
        // PID tuned for rotation units (0-1 range) instead of ticks (0-4096)
        m_azimuthMotor.pid0.setP(0.0336).setD(0.00042);
        m_azimuthMotor.usePIDSlot(PIDSlot.SLOT0);
        m_azimuthMotor.setAbsoluteWrapping(true);
        break;

      case SRX_MAG_ENCODER:
        m_azimuthMotor.setExternalEncoder(ExternalEncoder.SRX_MAG_ENCODER);
        // PID tuned for rotation units (0-1 range) instead of ticks (0-4096)
        m_azimuthMotor.pid0.setP(0.0336).setD(0.00126);
        m_azimuthMotor.usePIDSlot(PIDSlot.SLOT0);
        break;

      case REV_ENCODER:
        m_azimuthMotor.setExternalEncoder(ExternalEncoder.REV_ENCODER);
        // PID tuned for rotation units (0-1 range) instead of ticks (0-4096)
        m_azimuthMotor.pid0.setP(0.0336).setD(0.00126);
        m_azimuthMotor.usePIDSlot(PIDSlot.SLOT0);
        break;

      case THRIFTY_ABSOLUTE_ENCODER:
        // No motor controller configuration needed - using RIO PID
        break;

      case THRIFTY_10PIN_ENCODER:
        m_azimuthMotor.usePIDSlot(PIDSlot.SLOT0);
        m_azimuthMotor.setExternalEncoder(ExternalEncoder.THRIFTY_10_PIN_ENCODER);
        // PID tuned for rotation units (0-1 range) instead of ticks (0-4096)
        // P scaled up by encoder ticks per revolution (~4096x)
        m_azimuthMotor.pid0.setP(0.00336).setD(0.00326).setFF(0.0).setAllowableError(0.0042);
        m_azimuthMotor.setBrakeMode(true);
        m_azimuthMotor.setAbsoluteWrapping(true);
        break;
      default:
        System.err.println("Unknown encoder type for " + m_moduleName);
        break;
    }

    // For controller-handled encoders (Redux/SRX/REV), invert motor direction.
    if (ModuleConstants.ENCODER_SELECTED
        != DrivebaseConstants.EncoderType.THRIFTY_ABSOLUTE_ENCODER) {
      m_azimuthMotor.setInversion(m_azimuthInverted);
    }
  }

  /** Initialize the encoder offset, prioritizing saved values over constants */
  private void initializeOffset(double constantsOffsetTicks) {
    if (ModuleConstants.ENCODER_SELECTED
        == DrivebaseConstants.EncoderType.THRIFTY_ABSOLUTE_ENCODER) {
      // Thrifty encoder: Use constants for initial setup, will check saved values
      // later in periodic
      m_encoderOffsetTicks = constantsOffsetTicks;
      System.out.println(
          m_moduleName
              + " Thrifty encoder initialized with constants: "
              + ticksToDegrees(constantsOffsetTicks)
              + " degrees");
    } else {
      // Redux/SRX/REV: Set constants to Nova if provided, otherwise Nova will use
      // whatever is saved
      if (constantsOffsetTicks != 0) {
        m_azimuthMotor.setAbsOffset((int) constantsOffsetTicks);
        System.out.println(
            m_moduleName
                + " Nova encoder set to constants: "
                + ticksToDegrees(constantsOffsetTicks)
                + " degrees");
      } else {
        System.out.println(m_moduleName + " Nova encoder using saved offset");
      }

      // Nova handles offset automatically, Java doesn't need to track it
      m_encoderOffsetTicks = 0;
    }
  }

  /** Get raw encoder reading in ticks */
  private double getRawEncoderTicks() {
    double raw;
    if (ModuleConstants.ENCODER_SELECTED
        == DrivebaseConstants.EncoderType.THRIFTY_ABSOLUTE_ENCODER) {
      raw = m_thriftyEncoder.get();
    } else {
      raw = m_azimuthMotor.getPositionAbs() * m_encoderTicksPerRevolution;
    }
    // For THRIFTY_ABSOLUTE_ENCODER, use azimuthInverted to invert
    // sensor phase when required; otherwise, motor controller manages this.
    if (ModuleConstants.ENCODER_SELECTED == DrivebaseConstants.EncoderType.THRIFTY_ABSOLUTE_ENCODER
        && m_azimuthInverted) {
      return -raw;
    }
    return raw;
  }

  /** Get current encoder position in radians, accounting for offset */
  public double getEncoderPosition() {
    double rawTicks = getRawEncoderTicks();

    if (ModuleConstants.ENCODER_SELECTED
        == DrivebaseConstants.EncoderType.THRIFTY_ABSOLUTE_ENCODER) {
      // Thrifty encoder: motor controller doesn't apply offset, so we do it in
      // software
      double adjustedTicks = rawTicks - m_encoderOffsetTicks;
      return ticksToRadians(adjustedTicks);
    } else {
      return ticksToRadians(rawTicks);
    }
  }

  /** Get current swerve module state */
  public SwerveModuleState getSwerveState() {
    double velocityRevPerSec = m_driveMotor.getVelocity().getValueAsDouble();
    double velocityMPS =
        velocityRevPerSec
            * (DrivebaseConstants.WHEEL_DIAMETER_METERS * Math.PI)
            / DrivebaseConstants.DRIVE_GEAR_RATIO;
    return new SwerveModuleState(velocityMPS, new Rotation2d(getEncoderPosition()));
  }

  /** Get current swerve module position */
  public SwerveModulePosition getSwervePosition() {
    double positionRotations = m_driveMotor.getPosition().getValueAsDouble();
    double positionMeters =
        positionRotations
            * (DrivebaseConstants.WHEEL_DIAMETER_METERS * Math.PI)
            / DrivebaseConstants.DRIVE_GEAR_RATIO;
    return new SwerveModulePosition(positionMeters, new Rotation2d(getEncoderPosition()));
  }

  /** Set the desired state for this swerve module */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Skip optimization and hold position when not actually moving
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
      m_driveMotor.set(0);
      return;
    }

    Rotation2d currentAngle = new Rotation2d(getEncoderPosition());
    desiredState = SwerveModuleState.optimize(desiredState, currentAngle);

    double targetRevPerSec =
        desiredState.speedMetersPerSecond
            / (DrivebaseConstants.WHEEL_DIAMETER_METERS * Math.PI)
            * DrivebaseConstants.DRIVE_GEAR_RATIO;
    m_driveMotor.setControl(new VelocityVoltage(targetRevPerSec));

    m_desiredAngle = desiredState.angle;
    setAzimuthPosition(desiredState.angle.getRadians());
  }

  /** Set azimuth motor to target angle in radians */
  private void setAzimuthPosition(double targetAngleRadians) {
    if (ModuleConstants.ENCODER_SELECTED
        == DrivebaseConstants.EncoderType.THRIFTY_ABSOLUTE_ENCODER) {
      double currentAngle = getEncoderPosition();
      double output = m_turningPID.calculate(currentAngle, targetAngleRadians);
      m_azimuthMotor.set(output);
    } else {
      // Convert radians to rotations (0-1 range)
      double targetRotations = targetAngleRadians / (2 * Math.PI);
      // Normalize to 0-1 range
      targetRotations = ((targetRotations % 1.0) + 1.0) % 1.0;
      // Log the target rotations
      System.out.println("Target rotations: " + targetRotations);
      m_azimuthMotor.setPositionAbs(targetRotations);
    }
  }

  /** Set current position as the new zero offset */
  public void setZeroOffset() {
    double currentRawTicks = getRawEncoderTicks();

    if (ModuleConstants.ENCODER_SELECTED
        == DrivebaseConstants.EncoderType.THRIFTY_ABSOLUTE_ENCODER) {
      // Thrifty encoder: Read through RoboRIO AnalogEncoder, not Nova
      // Java does "position = raw - offset"
      // To make current position = 0: 0 = raw - offset, so offset = raw
      m_encoderOffsetTicks = currentRawTicks;

      // Save to Nova for persistence, but Nova doesn't use it for Thrifty encoder
      // control
      // (This is just for storing the value between code deploys)
      // setAbsOffset expects ticks (as int)
      m_azimuthMotor.setAbsOffset((int) currentRawTicks);

      System.out.println(
          m_moduleName
              + " Thrifty encoder zeroed. Java offset: "
              + currentRawTicks
              + " ticks ("
              + ticksToDegrees(currentRawTicks)
              + " degrees)");
    } else {
      // Redux/SRX/REV: Nova automatically applies offset to readings
      // Nova does "position = raw + offset"
      // To make current position = 0: 0 = raw + offset, so offset = -raw
      // setAbsOffset expects ticks (as int)
      int newOffset = -(int) currentRawTicks;
      m_azimuthMotor.setAbsOffset(newOffset);

      System.out.println(
          m_moduleName
              + " Nova encoder zeroed. Raw: "
              + (int) currentRawTicks
              + " ticks, Nova offset: "
              + newOffset
              + " ticks");
    }

    // Verify the zero worked by checking position
    System.out.println(
        m_moduleName
            + " Position after zero: "
            + Math.toDegrees(getEncoderPosition())
            + " degrees");
  }

  /** Stop both motors */
  public void stop() {
    m_driveMotor.set(0.0);
    m_azimuthMotor.set(0.0);
  }

  /** Update SmartDashboard with module information and handle zero button */
  public void updateSmartDashboard() {
    // Display encoder values
    Logger.recordOutput(m_moduleName + " Raw Encoder (ticks)", getRawEncoderTicks());
    Logger.recordOutput(m_moduleName + " Position (deg)", Math.toDegrees(getEncoderPosition()));
    Logger.recordOutput(m_moduleName + " Position (rad)", getEncoderPosition());
    Logger.recordOutput(m_moduleName + " desired angle (rad)", m_desiredAngle.getRadians());

    // Display offset info based on encoder type
    if (ModuleConstants.ENCODER_SELECTED
        == DrivebaseConstants.EncoderType.THRIFTY_ABSOLUTE_ENCODER) {
      Logger.recordOutput(m_moduleName + " Java Offset (ticks)", m_encoderOffsetTicks);
      Logger.recordOutput(
          m_moduleName + " Java Offset (deg)", ticksToDegrees(m_encoderOffsetTicks));
    }

    // Raw encoder readings from ThriftyNova (in rotations, 0-1 range)
    Logger.recordOutput(
        m_moduleName + " Raw getPositionAbs (rotations)", m_azimuthMotor.getPositionAbs());
    Logger.recordOutput(
        m_moduleName + " Raw getPosition (rotations)", m_azimuthMotor.getPosition());
    Logger.recordOutput(
        m_moduleName + " Raw getPositionInternal (rotations)",
        m_azimuthMotor.getPositionInternal());
    Logger.recordOutput(
        m_moduleName + " Raw as Ticks (x4096)",
        m_azimuthMotor.getPositionAbs() * m_encoderTicksPerRevolution);

    // Display drive motor info
    Logger.recordOutput(m_moduleName + " Drive Velocity", getSwerveState().speedMetersPerSecond);
    Logger.recordOutput(m_moduleName + " Drive Position", getSwervePosition().distanceMeters);

    // Display motor controller health
    Logger.recordOutput(
        m_moduleName + " Drive Current Output", m_driveMotor.getTorqueCurrent().getValueAsDouble());
    Logger.recordOutput(
        m_moduleName + " Drive Temperature", m_driveMotor.getDeviceTemp().getValueAsDouble());
    Logger.recordOutput(
        m_moduleName + " Azimuth Current Output", m_azimuthMotor.getStatorCurrent());
    Logger.recordOutput(m_moduleName + " Azimuth Current Output", m_azimuthMotor.getTemperature());
  }

  // Utility methods for unit conversions
  private double ticksToRadians(double ticks) {
    return (ticks / m_encoderTicksPerRevolution) * (2 * Math.PI);
  }

  private double ticksToDegrees(double ticks) {
    return (ticks / m_encoderTicksPerRevolution) * 360.0;
  }

  public void logSwerveModule() {
    Logger.recordOutput(m_moduleName + " Raw Encoder (ticks)", getRawEncoderTicks());
    Logger.recordOutput(m_moduleName + " Position (deg)", Math.toDegrees(getEncoderPosition()));
    Logger.recordOutput(m_moduleName + " Position (rad)", getEncoderPosition());
    Logger.recordOutput(m_moduleName + " desired angle (rad)", m_desiredAngle.getRadians());

    // Display offset info based on encoder type
    if (ModuleConstants.ENCODER_SELECTED
        == DrivebaseConstants.EncoderType.THRIFTY_ABSOLUTE_ENCODER) {
      Logger.recordOutput(m_moduleName + " Java Offset (ticks)", m_encoderOffsetTicks);
      Logger.recordOutput(
          m_moduleName + " Java Offset (deg)", ticksToDegrees(m_encoderOffsetTicks));
    }

    // Raw encoder readings from ThriftyNova (in rotations, 0-1 range)
    Logger.recordOutput(
        m_moduleName + " Raw getPositionAbs (rotations)", m_azimuthMotor.getPositionAbs());
    Logger.recordOutput(
        m_moduleName + " Raw getPosition (rotations)", m_azimuthMotor.getPosition());
    Logger.recordOutput(
        m_moduleName + " Raw getPositionInternal (rotations)",
        m_azimuthMotor.getPositionInternal());
    Logger.recordOutput(
        m_moduleName + " Raw as Ticks (x4096)",
        m_azimuthMotor.getPositionAbs() * m_encoderTicksPerRevolution);

    // Display drive motor info
    Logger.recordOutput(m_moduleName + " Drive Velocity", getSwerveState().speedMetersPerSecond);
    Logger.recordOutput(m_moduleName + " Drive Position", getSwervePosition().distanceMeters);

    // Display motor controller health
    Logger.recordOutput(
        m_moduleName + " Drive Current Output", m_driveMotor.getTorqueCurrent().getValueAsDouble());
    Logger.recordOutput(
        m_moduleName + " Drive Temperature", m_driveMotor.getDeviceTemp().getValueAsDouble());
    Logger.recordOutput(
        m_moduleName + " Azimuth Current Output", m_azimuthMotor.getStatorCurrent());
    Logger.recordOutput(m_moduleName + " Azimuth Current Output", m_azimuthMotor.getTemperature());
  }
}
