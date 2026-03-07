package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {
  private final Shooter m_ShooterSubsystem;

  private final Joystick ShootJoystick = new Joystick(OperatorConstants.COPILOT_CONTROLLER);

  public ShooterCommand(Shooter subsystem) {
    m_ShooterSubsystem = subsystem;
    addRequirements(m_ShooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Button 1 is simply a placeholder button, probably will be changed
    if (ShootJoystick.getRawButton(1)) {
      m_ShooterSubsystem.ShootStart();
      System.out.println("Shooting!");
      withTimeout(2);
      m_ShooterSubsystem.Shoot();
    } else if (ShootJoystick.getRawButton(12)) {
      m_ShooterSubsystem.StopShoot();
    }
  }
}
