package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Shooter;
import org.littletonrobotics.junction.Logger;

public class ShooterCommand extends Command {
  private final Shooter m_ShooterSubsystem;

  private final Joystick ShootJoystick = new Joystick(OperatorConstants.kCOPILOT_CONTROLLER);

  private boolean isRunning = false;

  public ShooterCommand(Shooter subsystem) {
    m_ShooterSubsystem = subsystem;
    addRequirements(m_ShooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Check if buttons 5 and 6 are pressed, change a toggle
    // Button 5 will set the shooter to run, Button 6 will stop it
    if (DriverStation.isAutonomousEnabled()) {
      isRunning = true;
    }

    if (ShootJoystick.getRawButton(5)) {
      isRunning = true;
    } else if (ShootJoystick.getRawButton(6)) {
      isRunning = false;
    }

    // Our main toggle system for running the motors
    if (isRunning) {
      m_ShooterSubsystem.Shoot();
    } else if (!isRunning) {
      m_ShooterSubsystem.StopShoot();
    }

    Logger.recordOutput("Shooter/", isRunning);
  }
}
