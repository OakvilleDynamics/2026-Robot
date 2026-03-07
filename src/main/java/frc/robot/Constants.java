// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);

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
    public static final double WHEEL_LOCK_TIME = 10; // seconds
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

    // REV Power Distribution Hub CAN ID
    public static final int REV_PDH_ID = 10;
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
