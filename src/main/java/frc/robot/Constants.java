// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /** Constants related to the drivebase, such as dimensions */
  public static final class DrivebaseConstants {
    // Drive base dimensions
    public static final double DRIVE_BASE_WIDTH_METERS = Units.inchesToMeters(24);
    public static final double DRIVE_BASE_LENGTH_METERS = Units.inchesToMeters(24);

    // Gear ratios
    public static final double DRIVE_GEAR_RATIO = 5.79;
    public static final double AZIMUTH_GEAR_RATIO = 24;

    // Wheel specifications
    public static final double WHEEL_DIAMETER_INCHES = 4;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);

    // Top Speed (calculated from motor free speed, wheel diameter & gear ratio)
    public static final double TOP_SPEED_MOTOR_RPM = 6000;
    public static final double TOP_SPEED_METERS_PER_SEC =
        TOP_SPEED_MOTOR_RPM
            * WHEEL_DIAMETER_METERS
            * Math.PI
            / DRIVE_GEAR_RATIO
            / 60; // = 4.729 m/s

    // Module locations from center of robot
    private static final double HALF_WIDTH = DRIVE_BASE_WIDTH_METERS / 2.0;
    private static final double HALF_LENGTH = DRIVE_BASE_LENGTH_METERS / 2.0;

    // Kinematics
    public static final Translation2d FRONT_LEFT_LOCATION =
        new Translation2d(HALF_LENGTH, HALF_WIDTH);
    public static final Translation2d FRONT_RIGHT_LOCATION =
        new Translation2d(HALF_LENGTH, -HALF_WIDTH);
    public static final Translation2d BACK_LEFT_LOCATION =
        new Translation2d(-HALF_LENGTH, HALF_WIDTH);
    public static final Translation2d BACK_RIGHT_LOCATION =
        new Translation2d(-HALF_LENGTH, -HALF_WIDTH);

    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
        new SwerveDriveKinematics(
            FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION);

    // Encoder type enum - MUST be defined before ModuleConstants
    public static enum EncoderType {
      REDUX_ENCODER,
      SRX_MAG_ENCODER,
      REV_ENCODER,
      THRIFTY_ABSOLUTE_ENCODER,
      THRIFTY_10PIN_ENCODER
    }

    public static enum GyroType {
      NAVX,
      PIGEON2,
      CANANDGYRO,
      NONE
    }

    // Module constants
    public static final class ModuleConstants {
      // Front Left Module
      public static final class FrontLeftModule {
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 2;
        public static final int FRONT_LEFT_AZIMUTH_MOTOR_ID = 3;
        public static final int FRONT_LEFT_ENCODER_PORT =
            0; // RoboRIO analog input port (for Thrifty absolute encoder)
        public static final double FRONT_LEFT_ENCODER_OFFSET = 3118; // Encoder offset in ticks
        public static final boolean FRONT_LEFT_DRIVE_MOTOR_INVERTED = false;
        public static final boolean FRONT_LEFT_AZIMUTH_MOTOR_INVERTED = false;
      }

      // Front Right Module
      public static final class FrontRightModule {
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 4;
        public static final int FRONT_RIGHT_AZIMUTH_MOTOR_ID = 5;
        public static final int FRONT_RIGHT_ENCODER_PORT =
            1; // RoboRIO analog input port (for Thrifty absolute encoder)
        public static final double FRONT_RIGHT_ENCODER_OFFSET = 2651;
        public static final boolean FRONT_RIGHT_DRIVE_MOTOR_INVERTED = false;
        public static final boolean FRONT_RIGHT_AZIMUTH_MOTOR_INVERTED = false;
      }

      // Back Left Module
      public static final class BackLeftModule {
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 6;
        public static final int BACK_LEFT_AZIMUTH_MOTOR_ID = 7;
        public static final int BACK_LEFT_ENCODER_PORT =
            2; // RoboRIO analog input port (for Thrifty absolute encoder)
        public static final double BACK_LEFT_ENCODER_OFFSET = 934; // Encoder offset in ticks
        public static final boolean BACK_LEFT_DRIVE_MOTOR_INVERTED = false;
        public static final boolean BACK_LEFT_AZIMUTH_MOTOR_INVERTED = false;
      }

      // Back Right Module
      public static final class BackRightModule {

        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 8;
        public static final int BACK_RIGHT_AZIMUTH_MOTOR_ID = 9;
        public static final int BACK_RIGHT_ENCODER_PORT =
            3; // RoboRIO analog input port (for Thrifty absolute encoder)
        public static final double BACK_RIGHT_ENCODER_OFFSET = 3834; // Encoder offset in ticks
        public static final boolean BACK_RIGHT_DRIVE_MOTOR_INVERTED = false;
        public static final boolean BACK_RIGHT_AZIMUTH_MOTOR_INVERTED = false;
      }

      public static final EncoderType ENCODER_SELECTED = EncoderType.THRIFTY_ABSOLUTE_ENCODER;
      public static final double ENCODER_TICKS_PER_ROTATION = 4096;
    }

    // Gyro constants
    public static final class GyroConstants {
      public static final GyroType GYRO_TYPE = GyroType.PIGEON2;
      public static final Object[] GYRO_PARAMS = new Object[] {}; // No parameters needed for NavX
    }
  }

  /**
   * Constants related to the operator interface, such as controller ports, deadbands, and other
   * constants related to driver input
   */
  public static class OperatorConstants {
    // Controller ports
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int CO_DRIVER_CONTROLLER_PORT = 1;

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
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
      public static final String KENOBI_RIO_SERIAL = "0332053D";
      public static final String VADER_RIO_SERIAL = "033205CD";

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
        if (serial.equals(KENOBI_RIO_SERIAL)) {
          return RioSerials.KENOBI_RIO_SERIAL;
        } else if (serial.equals(VADER_RIO_SERIAL)) {
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

    // REV Power Distribution Hub CAN ID, this is only used for the Kenobi drivebase.
    public static final int REV_PDH_ID = 10;

    // CTRE Power Distribution Panel CAN ID, this is only used for the Vader drivebase.
    public static final int CTRE_PDP_ID = 20;
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
}
