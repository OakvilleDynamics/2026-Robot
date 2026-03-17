// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double kROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter kCHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), kROBOT_MASS);
  public static final double kLOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double kMAX_SPEED = Units.feetToMeters(14.5);

  // Maximum speed of the robot in meters per second, used to limit acceleration.

  //  public static final class AutonConstants
  //  {
  //
  //    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
  //    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  //  }

  /** Constants related to the drivebase, such as dimensions */
  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double kWHEEL_LOCK_TIME = 10; // seconds
  }

  /**
   * Constants related to the operator interface, such as controller ports, deadbands, and other
   * constants related to driver input
   */
  public static class OperatorConstants {
    // Controller ports
    public static final int kDRIVER_CONTROLLER = 0;
    public static final int kCOPILOT_CONTROLLER = 1;

    // Joystick Deadband
    public static final double kDEADBAND = 0.1;
    public static final double kLEFT_Y_DEADBAND = 0.1;
    public static final double kRIGHT_X_DEADBAND = 0.1;
    public static final double kTURN_CONSTANT = 6;
  }

  /**
   * Constants related to hardware, such as CAN IDs, serial numbers, and other constants related to
   * the physical robot
   *
   * <p>Note: these should be constants that are not expected to change between different physical
   * robots, such as the Kenobi and Vader drivebases.
   *
   * <p>Constants that are expected to change between different physical robots should be placed in
   * the RobotContainer class, where they can be set at runtime based on the detected hardware.
   */
  public static class HardwareConstants {

    public static class RioState {
      public static final String kKENOBI_RIO_SERIAL = "0332053D";
      public static final String kVADER_RIO_SERIAL = "033205CD";

      /**
       * Gets the RoboRIO serial number and returns an enum representing which RoboRIO is currently
       * in use. This allows us to select different configurations at runtime based on the detected
       * hardware, such as selecting the correct drivetrain configuration JSON for the Kenobi and
       * Vader drivebases.
       *
       * @return an enum representing which RoboRIO is currently in use
       */
      public static RioSerials getRioSerial() {
        String serial = RobotController.getSerialNumber();
        if (serial.equals(kKENOBI_RIO_SERIAL)) {
          return RioSerials.KENOBI_RIO_SERIAL;
        } else if (serial.equals(kVADER_RIO_SERIAL)) {
          return RioSerials.VADER_RIO_SERIAL;
        } else {
          return RioSerials.UNKNOWN;
        }
      }

      public static enum RioSerials {
        /** KENOBI RIO Serial */
        KENOBI_RIO_SERIAL,
        /** VADER RIO Serial */
        VADER_RIO_SERIAL,
        /** Unknown RIO Serial */
        UNKNOWN
      }
    }

    // REV Power Distribution Hub CAN ID
    public static final int kREV_PDH_ID = 10;
  }

  public static class RuntimeConstants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
      /** Running on a real robot. */
      REAL,

      /** Running a physics simulator. */
      SIM,

      /** Replaying from a log file. */
      REPLAY
    }
  }

  /** Constants related to mechanisms, such as CAN IDs, motor speeds, and PID constants */
  public static class MechanismConstants {
    // Subsystem CAN IDs
    public static final int kIntakeMotor = 11;
    public static final int kIntakeHinge = 12;
    public static final int kShooterMotor = 13;
    public static final int kIndexMotor = 14;
    public static final int ClimberMotor = 21;

    /** Constants for the shooter mechanism. */
    public static class ShooterConstants {
      public static final double kShooter_Speed1 = 0.8;
      public static final double kShooter_Speed2 = 0.4;
      public static final InvertedValue kShooterMotor_Inverted = InvertedValue.Clockwise_Positive;

      // Static gain, the minimum voltage needed to overcome friction and start the motor moving
      public static final double kShooter_kS = 0.2;
      // Velocity gain, the voltage needed to maintain a certain velocity (V = kV * velocity)
      public static final double kShooter_kV = 0.1;
      // Proportional gain, the voltage added based on the error between the target
      // velocity and the actual velocity (V = kP * error)
      public static final double kShooter_kP = 0.5;
      // Integral gain, the voltage added based on the accumulated error over time (V = kI *
      // integral of error)
      public static final double kShooter_kI = 0.001;
      // Derivative gain, the voltage added based on the rate of change of the error (V = kD *
      // derivative of error)
      public static final double kShooter_kD = 5;
    }

    /** Constants for the indexer mechanism. */
    public static class IndexerConstants {
      public static final double kIndex_Speed = 0.3;
      public static final boolean kIndex_Inverted = false;

      public static final SparkFlexConfig kIndexerConfig = new SparkFlexConfig();

      static {
        kIndexerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)
            .voltageCompensation(12)
            .openLoopRampRate(0.5);
      }
    }

    /** Constants for the intake mechanism. */
    public static class IntakeConstants {
      public static final double kIntake_Speed = 0.5;
      public static final double kIntake_Hinge_Speed = 0.5;
      public static final boolean kIntakeMotor_Inverted = false;
      public static final boolean kIntakeHinge_Inverted = false;

      public static final SparkFlexConfig kIntakeMotorConfig = new SparkFlexConfig();

      static {
        kIntakeMotorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)
            .voltageCompensation(12)
            .openLoopRampRate(0.5);
      }

      public static final SparkFlexConfig kIntakeHingeConfig = new SparkFlexConfig();

      static {
        kIntakeHingeConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)
            .voltageCompensation(12)
            .openLoopRampRate(0.5);
      }
    }

    /** Constants for the climber mechanism. */
    public static class ClimberConstants {
      public static final boolean ClimberMotor_Inverted = false;
      public static final double Climber_Speed = 1.0;
    }
  }
}
