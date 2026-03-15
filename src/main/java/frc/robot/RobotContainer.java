// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swerve.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController m_Controller =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  final CommandXboxController coDriverXbox =
      new CommandXboxController(OperatorConstants.CO_DRIVER_CONTROLLER_PORT);

  // Gyro supplier created via factory and constants
  private final GyroSupplier m_gyro =
      GyroFactory.createGyro(
          DrivebaseConstants.GyroConstants.GYRO_TYPE, DrivebaseConstants.GyroConstants.GYRO_PARAMS);

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing
  // selection of desired auto
  //   private final SendableChooser<Command> autoChooser;
  // Driver controller
  // Swerve drivetrain subsystem
  private final Drivetrain m_swerve = new Drivetrain(m_gyro::getRotation2d, new Pose2d());
  // private final SimDrivetrain m_simSwerve = new SimDrivetrain(new Pose2d());

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(20);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(20);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(20);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // Have the autoChooser pull in all PathPlanner autos as options
    // autoChooser = AutoBuilder.buildAutoChooser();

    // Set the default auto (do nothing)
    // autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // Put the autoChooser on the SmartDashboard
    // SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {}

  private void configureDefaultCommands() {
    // Default drive command: run every scheduler cycle in teleop
    m_swerve.setDefaultCommand(
        new RunCommand(
            () -> {
              // Get the x speed. We are inverting this because Xbox controllers return
              // negative values when we push forward.
              final var xSpeed =
                  -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_Controller.getLeftY(), 0.05))
                      * DrivebaseConstants.TOP_SPEED_METERS_PER_SEC;

              // Get the y speed or sideways/strafe speed. We are inverting this because
              // we want a positive value when we pull to the left. Xbox controllers
              // return positive values when you pull to the right by default.
              final var ySpeed =
                  -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_Controller.getLeftX(), 0.05))
                      * DrivebaseConstants.TOP_SPEED_METERS_PER_SEC;

              // Get the rate of angular rotation. We are inverting this because we want a
              // positive value when we pull to the left (remember, CCW is positive in
              // mathematics). Xbox controllers return positive values when you pull to
              // the right by default.
              final var rot =
                  -m_rotLimiter.calculate(MathUtil.applyDeadband(m_Controller.getRightX(), 0.05))
                      * Drivetrain.kMaxAngularSpeed;

              // Command the drivetrain. 0.02 is the nominal TimedRobot loop period (20 ms).
              m_swerve.drive(xSpeed, ySpeed, rot, true, 0.02);
              // m_simSwerve.drive(xSpeed, ySpeed, rot, true, 0.02);
            },
            m_swerve));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //   public Command getAutonomousCommand() {
  //     // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand
  //     // return autoChooser.getSelected();
  //   }
}
