package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Constants.DrivebaseConstants.GyroConstants;

public class Gyro {
  private static Pigeon2 m_pigeon;

  public Gyro() {
    // Private constructor to prevent instantiation
    m_pigeon =
        new Pigeon2(
            GyroConstants.PIGEON2_CAN_ID); // Initialize the Pigeon2 gyro with CAN ID from constants
  }

  public static Pigeon2 getInstance() {
    if (m_pigeon == null) {
      new Gyro(); // Initialize the singleton instance
    }
    return m_pigeon;
  }

  /**
   * Provides access to the Pigeon2 gyro for use in commands that need it (like auto builders or
   * custom commands that use the gyro for heading control).
   *
   * @return The Pigeon2 gyro instance used by the drivetrain
   */
  public static Pigeon2 getPigeon() {
    return getInstance(); // Return the singleton instance of the Pigeon2 gyro
  }
}
