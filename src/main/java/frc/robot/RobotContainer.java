// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HardwareConstants.RioState;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  final CommandXboxController coDriverXbox =
      new CommandXboxController(OperatorConstants.CO_DRIVER_CONTROLLER_PORT);

  // The path to the drivetrain configuration json file, selected based on the RoboRIO serial number
  // of the robot. This allows us to use the same codebase for both the competition robot and the
  // practice robot, which have different swerve configurations.
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), selectDrivetrain()));

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing
  // selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driverXbox.getLeftY() * -1,
              () -> driverXbox.getLeftX() * -1)
          .withControllerRotationAxis(driverXbox::getRightX)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** Clone's the angular velocity input stream and converts it to a fieldRelative input stream. */
  SwerveInputStream driveDirectAngle =
      driveAngularVelocity
          .copy()
          .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
          .headingWhile(true);

  /** Clone's the angular velocity input stream and converts it to a robotRelative input stream. */
  SwerveInputStream driveRobotOriented =
      driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

  // Keyboard controls for simulation
  SwerveInputStream driveAngularVelocityKeyboard =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX())
          .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard =
      driveAngularVelocityKeyboard
          .copy()
          .withControllerHeadingAxis(
              () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
              () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
          .headingWhile(true)
          .translationHeadingOffset(true)
          .translationHeadingOffset(Rotation2d.fromDegrees(0));

  // Test input stream for debugging
  SwerveInputStream test =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driverXbox.getLeftY() * -1,
              () -> driverXbox.getLeftX() * -1)
          .withControllerRotationAxis(() -> driverXbox.getRightX())
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.9)
          .allianceRelativeControl(true);

  // YAGSL 8 steps debugging swerve input stream
  Command drive8StepsConfig =
      drivebase.driveCommand(
          () -> MathUtil.applyDeadband((driverXbox.getLeftY() * -1), OperatorConstants.DEADBAND),
          () -> MathUtil.applyDeadband((driverXbox.getLeftX() * -1), OperatorConstants.DEADBAND),
          () -> driverXbox.getRightX(),
          () -> driverXbox.getRightY());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    // Set the default auto (do nothing)
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

    // Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // For testing: put the PIDF values of the first module's angle controller on the
    // SmartDashboard, as we are not setting per-module PIDs
    SmartDashboard.putNumber(
        "YAGSL Angle PID P", drivebase.getSwerveDrive().getModules()[0].getAnglePIDF().p);
    SmartDashboard.putNumber(
        "YAGSL Angle PID I", drivebase.getSwerveDrive().getModules()[0].getAnglePIDF().i);
    SmartDashboard.putNumber(
        "YAGSL Angle PID D", drivebase.getSwerveDrive().getModules()[0].getAnglePIDF().d);
    SmartDashboard.putNumber(
        "YAGSL Angle PID F", drivebase.getSwerveDrive().getModules()[0].getAnglePIDF().f);
    SmartDashboard.putNumber(
        "YAGSL Angle PID IZ", drivebase.getSwerveDrive().getModules()[0].getAnglePIDF().iz);
    SmartDashboard.putNumber(
        "YAGSL Drive PID P", drivebase.getSwerveDrive().getModules()[0].getDrivePIDF().p);
    SmartDashboard.putNumber(
        "YAGSL Drive PID I", drivebase.getSwerveDrive().getModules()[0].getDrivePIDF().i);
    SmartDashboard.putNumber(
        "YAGSL Drive PID D", drivebase.getSwerveDrive().getModules()[0].getDrivePIDF().d);
    SmartDashboard.putNumber(
        "YAGSL Drive PID F", drivebase.getSwerveDrive().getModules()[0].getDrivePIDF().f);
    SmartDashboard.putNumber(
        "YAGSL Drive PID IZ", drivebase.getSwerveDrive().getModules()[0].getDrivePIDF().iz);

    Logger.recordOutput("Angle PID P", drivebase.getSwerveDrive().getModules()[0].getAnglePIDF().p);
    Logger.recordOutput("Angle PID I", drivebase.getSwerveDrive().getModules()[0].getAnglePIDF().i);
    Logger.recordOutput("Angle PID D", drivebase.getSwerveDrive().getModules()[0].getAnglePIDF().d);
    Logger.recordOutput("Angle PID F", drivebase.getSwerveDrive().getModules()[0].getAnglePIDF().f);
    Logger.recordOutput(
        "Angle PID IZ", drivebase.getSwerveDrive().getModules()[0].getAnglePIDF().iz);
    Logger.recordOutput("Drive PID P", drivebase.getSwerveDrive().getModules()[0].getDrivePIDF().p);
    Logger.recordOutput("Drive PID I", drivebase.getSwerveDrive().getModules()[0].getDrivePIDF().i);
    Logger.recordOutput("Drive PID D", drivebase.getSwerveDrive().getModules()[0].getDrivePIDF().d);
    Logger.recordOutput("Drive PID F", drivebase.getSwerveDrive().getModules()[0].getDrivePIDF().f);
    Logger.recordOutput(
        "Drive PID IZ", drivebase.getSwerveDrive().getModules()[0].getDrivePIDF().iz);
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
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard =
        drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard =
        drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard =
        drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    Command driveTestCommand = drivebase.driveFieldOriented(test);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveTestCommand);
    }

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
      // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(
          () -> target,
          new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
          new ProfiledPIDController(
              5, 0, 0, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));
      driverXbox
          .start()
          .onTrue(
              Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox
          .button(2)
          .whileTrue(
              Commands.runEnd(
                  () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                  () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

      driverXbox
          .b()
          .whileTrue(
              drivebase.driveToPose(
                  new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(
          driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand
    return autoChooser.getSelected();
  }

  /** Sets the motor brake mode. */
  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  /**
   * Selects the drivetrain configuration json file based on the RoboRIO serial number of the robot.
   * This allows us to use the same codebase for both the competition robot and the practice robot,
   * which have different swerve configurations.
   *
   * @return the path to the drivetrain configuration JSON path
   */
  public String selectDrivetrain() {
    if (RioState.getRioSerial() == RioState.RioSerials.VADER_RIO_SERIAL) {
      return "swerve/vader";
    } else if (RioState.getRioSerial() == RioState.RioSerials.KENOBI_RIO_SERIAL) {
      return "swerve/kenobi";
    } else {
      // If not found, default to the Kenobi configuration
      return "swerve/kenobi";
    }
  }
}
