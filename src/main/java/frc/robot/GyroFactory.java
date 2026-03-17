package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.DrivebaseConstants.GyroType;

interface GyroSupplier {
  Rotation2d getRotation2d();
}

public class GyroFactory {
  public static GyroSupplier createGyro(GyroType gyroType, Object... params) {
    switch (gyroType) {
      // case NAVX -> {
      //   // Check if parameters are provided, otherwise use defaults
      //   AHRS.NavXComType comType =
      //       params.length > 0 && params[0] instanceof AHRS.NavXComType
      //           ? (AHRS.NavXComType) params[0]
      //           : AHRS.NavXComType.kMXP_SPI;

      //   AHRS ahrs = new AHRS(comType);
      //   return ahrs::getRotation2d;
      // }

      case PIGEON2 -> {
        // Check if parameters are provided, otherwise use defaults
        int canID =
            params.length > 0 && params[0] instanceof Integer
                ? (Integer) params[0]
                : 1; // Default CAN ID

        Pigeon2 pigeon = new Pigeon2(canID);
        return pigeon::getRotation2d;
      }

      default -> throw new IllegalArgumentException("Unsupported gyro type: " + gyroType);
    }
  }
}
