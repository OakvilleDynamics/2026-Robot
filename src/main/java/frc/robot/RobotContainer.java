// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Shooter m_Shooter = new Shooter();
  private final Climber m_Climber = new Climber();
  private final Index m_Index = new Index();
  private final Intake m_Intake = new Intake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox =
      new CommandXboxController(OperatorConstants.kDRIVER_CONTROLLER);
  final CommandXboxController coDriverXbox =
      new CommandXboxController(OperatorConstants.kCOPILOT_CONTROLLER);

  // The path to the drivetrain configuration json file, selected based on the RoboRIO serial number
  // of
  // the robot. This allows us to use the same codebase for both the competition robot and the
  // practice robot, which have different swerve configurations.

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing
  // selection of desired auto

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   *
   * <p>SwerveInputStream driveAngularVelocity = SwerveInputStream.of( drivebase.getSwerveDrive(),
   * () -> driverXbox.getLeftY() * -1, () -> driverXbox.getLeftX() * -1)
   * .withControllerRotationAxis(driverXbox::getRightX) .deadband(OperatorConstants.kDEADBAND)
   * .scaleTranslation(0.8) .allianceRelativeControl(true);
   *
   * <p>/** Clone's the angular velocity input stream and converts it to a fieldRelative input
   * stream. SwerveInputStream driveDirectAngle = driveAngularVelocity .copy()
   * .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY) .headingWhile(true);
   *
   * <p>/** Clone's the angular velocity input stream and converts it to a robotRelative input
   * stream. SwerveInputStream driveRobotOriented =
   * driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);
   *
   * <p>// Keyboard controls for simulation SwerveInputStream driveAngularVelocityKeyboard =
   * SwerveInputStream.of( drivebase.getSwerveDrive(), () -> -driverXbox.getLeftY(), () ->
   * -driverXbox.getLeftX()) .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
   * .deadband(OperatorConstants.kDEADBAND) .scaleTranslation(0.8) .allianceRelativeControl(true);
   *
   * <p>// Derive the heading axis with math! SwerveInputStream driveDirectAngleKeyboard =
   * driveAngularVelocityKeyboard .copy() .withControllerHeadingAxis( () ->
   * Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2), () ->
   * Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2)) .headingWhile(true)
   * .translationHeadingOffset(true) .translationHeadingOffset(Rotation2d.fromDegrees(0));
   *
   * <p>/** The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Set default commands for subsystems
    m_Shooter.setDefaultCommand(new ShooterCommand(m_Shooter));
    m_Climber.setDefaultCommand(new ClimberCommand(m_Climber));
    m_Index.setDefaultCommand(new IndexCommand(m_Index));
    m_Intake.setDefaultCommand(new IntakeCommand(m_Intake));
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand
    return new Command() {};
  }
}
