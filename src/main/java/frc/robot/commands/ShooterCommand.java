package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {
  private final Shooter m_ShooterSubsystem;

  private final Joystick ShootJoystick = new Joystick(OperatorConstants.kCOPILOT_CONTROLLER);

  public ShooterCommand(Shooter subsystem) {
    m_ShooterSubsystem = subsystem;
    addRequirements(m_ShooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Button 1 is simply a placeholder button, probably will be changed
    if (!DriverStation.isFMSAttached() && DriverStation.isTeleop()) {    
      if (ShootJoystick.getRawButton(5)) {
        m_ShooterSubsystem.SpinUpShooter();
        m_ShooterSubsystem.Shoot();
      } else if (ShootJoystick.getRawButton(6)) {
        m_ShooterSubsystem.StopShoot();
      }
    } else if (DriverStation.isFMSAttached()) {
      m_ShooterSubsystem.Shoot();
    }
  }
}
