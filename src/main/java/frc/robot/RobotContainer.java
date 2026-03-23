// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Drivetrain;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Shooter m_Shooter = new Shooter();
  private final Climber m_Climber = new Climber();
  private final Index m_Index = new Index();
  private final Intake m_Intake = new Intake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController m_Driver_Controller =
      new CommandXboxController(OperatorConstants.kDRIVER_CONTROLLER);
  final CommandJoystick m_Copilot_Controller =
      new CommandJoystick(OperatorConstants.kCOPILOT_CONTROLLER);

  // Gyro supplier created via factory and constants
  private final GyroSupplier m_gyro =
      GyroFactory.createGyro(
          DrivebaseConstants.GyroConstants.GYRO_TYPE, DrivebaseConstants.GyroConstants.GYRO_PARAMS);
  // Swerve drivetrain subsystem
  private final Drivetrain m_swerve = new Drivetrain(m_gyro::getRotation2d, new Pose2d());
  // private final SimDrivetrain m_simSwerve = new SimDrivetrain(new Pose2d());

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(20);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(20);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(20);
  private LoggedDashboardChooser<Command> autoChooser;

  // Normal swerve drive command
  private final RunCommand normalDrive =
      new RunCommand(
          () -> {
            // Get the x speed. We are inverting this because Xbox controllers return
            // negative values when we push forward.
            final var xSpeed =
                -m_xspeedLimiter.calculate(
                        MathUtil.applyDeadband(
                            m_Driver_Controller.getLeftY(), OperatorConstants.kDEADBAND))
                    * DrivebaseConstants.TOP_SPEED_METERS_PER_SEC;

            // Get the y speed or sideways/strafe speed. We are inverting this because
            // we want a positive value when we pull to the left. Xbox controllers
            // return positive values when you pull to the right by default.
            final var ySpeed =
                -m_yspeedLimiter.calculate(
                        MathUtil.applyDeadband(
                            m_Driver_Controller.getLeftX(), OperatorConstants.kDEADBAND))
                    * DrivebaseConstants.TOP_SPEED_METERS_PER_SEC;

            // Get the rate of angular rotation. We are inverting this because we want a
            // positive value when we pull to the left (remember, CCW is positive in
            // mathematics). Xbox controllers return positive values when you pull to
            // the right by default.
            final var rot =
                -m_rotLimiter.calculate(
                        MathUtil.applyDeadband(
                            m_Driver_Controller.getRightX(), OperatorConstants.kDEADBAND))
                    * Drivetrain.kMaxAngularSpeed;

            // Command the drivetrain. 0.02 is the nominal TimedRobot loop period (20 ms).
            m_swerve.drive(xSpeed, ySpeed, rot, true, 0.02);
            // m_simSwerve.drive(xSpeed, ySpeed, rot, true, 0.02);
          },
          m_swerve);

  // Slowed swerve drive, for extra precision.
  private final RunCommand slowedDrive =
      new RunCommand(
          () -> {
            // Get the x speed. We are inverting this because Xbox controllers return
            // negative values when we push forward.
            final var xSpeed =
                (-m_xspeedLimiter.calculate(
                            MathUtil.applyDeadband(
                                m_Driver_Controller.getLeftY(), OperatorConstants.kDEADBAND))
                        * DrivebaseConstants.TOP_SPEED_METERS_PER_SEC)
                    * 0.35;

            // Get the y speed or sideways/strafe speed. We are inverting this because
            // we want a positive value when we pull to the left. Xbox controllers
            // return positive values when you pull to the right by default.
            final var ySpeed =
                (-m_yspeedLimiter.calculate(
                            MathUtil.applyDeadband(
                                m_Driver_Controller.getLeftX(), OperatorConstants.kDEADBAND))
                        * DrivebaseConstants.TOP_SPEED_METERS_PER_SEC)
                    * 0.35;

            // Get the rate of angular rotation. We are inverting this because we want a
            // positive value when we pull to the left (remember, CCW is positive in
            // mathematics). Xbox controllers return positive values when you pull to
            // the right by default.
            final var rot =
                (-m_rotLimiter.calculate(
                            MathUtil.applyDeadband(
                                m_Driver_Controller.getRightX(), OperatorConstants.kDEADBAND))
                        * Drivetrain.kMaxAngularSpeed)
                    * 0.35;

            // Command the drivetrain. 0.02 is the nominal TimedRobot loop period (20 ms).
            m_swerve.drive(xSpeed, ySpeed, rot, true, 0.02);
            // m_simSwerve.drive(xSpeed, ySpeed, rot, true, 0.02);
          },
          m_swerve);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();

    // Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("Climb", Commands.run(m_Climber::Climb, m_Climber));
    NamedCommands.registerCommand("Descend", Commands.run(m_Climber::Descend, m_Climber));
    // NamedCommands.registerCommand("Shoot", Commands.run(m_Shooter::Shoot, m_Shooter));

    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser =
        new LoggedDashboardChooser<Command>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set the default auto (do nothing)
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
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
  private void configureBindings() {
    // Driver Controller binds (Xbox)
    // A, B, X, Y buttons
    m_Driver_Controller.a().whileTrue(Commands.none());
    m_Driver_Controller.b().whileTrue(Commands.none());
    m_Driver_Controller.x().whileTrue(Commands.runOnce(m_swerve::setX, m_swerve).repeatedly());
    m_Driver_Controller.y().whileTrue(Commands.none());

    // Bumpers
    m_Driver_Controller.leftBumper().whileTrue(slowedDrive).onFalse(normalDrive);
    m_Driver_Controller.rightBumper().whileTrue(Commands.none());

    // POV (D-pad)
    m_Driver_Controller.povUp().whileTrue(Commands.none());
    m_Driver_Controller.povDown().whileTrue(Commands.none());
    m_Driver_Controller.povLeft().whileTrue(Commands.none());
    m_Driver_Controller.povRight().whileTrue(Commands.none());

    // Start/Back buttons
    m_Driver_Controller.start().whileTrue(Commands.none());
    m_Driver_Controller.back().onTrue(Commands.runOnce(m_swerve::zeroGyro, m_swerve));

    // Copilot Controller binds (Joystick)
    m_Copilot_Controller.trigger().whileTrue(Commands.runOnce(m_Intake::IntakeFuel, m_Intake).repeatedly());
    m_Copilot_Controller.top().whileTrue(Commands.runOnce(m_Intake::IntakeSpit, m_Intake).repeatedly());
    m_Copilot_Controller.button(3).whileTrue(Commands.runOnce(m_Index::IndexReverse, m_Index).repeatedly());
    m_Copilot_Controller.button(4).whileTrue(Commands.runOnce(m_Index::IndexMove, m_Index).repeatedly());
    m_Copilot_Controller.button(7).whileTrue(Commands.runOnce(m_Intake::IntakeUp, m_Intake).repeatedly());
    m_Copilot_Controller.button(8).whileTrue(Commands.runOnce(m_Intake::IntakeDown, m_Intake).repeatedly());
    m_Copilot_Controller.button(10).whileTrue(Commands.runOnce(m_Climber::Climb, m_Climber).repeatedly());
    m_Copilot_Controller.button(11).whileTrue(Commands.runOnce(m_Climber::Descend, m_Climber).repeatedly());
  }

  /** This method sets subsystem commands */
  private void configureDefaultCommands() {
    // Default drive command: run every scheduler cycle in teleop
    m_swerve.setDefaultCommand(normalDrive);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Pass in the selected auto from the SmartDashboard as our desired autonomous command
    return autoChooser.get();
  }
}
