// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int DRIVER_CONTROLLER = 0;
    public static final int COPILOT_CONTROLLER = 1;

    // Joystick Deadband ~~ Temporary placeholders until testing
    public static final double kDEADBAND = 0.1;
    public static final double kLEFT_Y_DEADBAND = 0.1;
    public static final double kRIGHT_X_DEADBAND = 0.1;
    public static final double kTURN_CONSTANT = 6;
  }

  public static class MechanismConstants {
    // Subsystem CAN IDs ~~ May change in the future
    public static final int ShooterMotor = 13;
    public static final int IntakeMotor = 14;
    public static final int IntakeHinge = 15;

    // Inverts
    public static final boolean ShooterMotor_Inverted = false;
    public static final boolean IntakeMotor_Inverted = false;
    public static final boolean IntakeHinge_Inverted = false;

    // Motor speeds ~~ Change as needed
    public static final double Shooter_Speed = 0.5;
    public static final double Intake_Speed = 0.5;
    public static final double Intake_Hinge_Speed = 0.5;
  }
}
