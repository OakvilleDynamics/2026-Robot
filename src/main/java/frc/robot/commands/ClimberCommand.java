package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends Command {

  private final Climber m_ClimberSubsystem;

  private final Joystick RelayJoystick = new Joystick(OperatorConstants.kCOPILOT_CONTROLLER);

  public ClimberCommand(Climber subsystem) {
    m_ClimberSubsystem = subsystem;
    addRequirements(m_ClimberSubsystem);
  }

  public void execute() {
    if (RelayJoystick.getRawButton(10)) {
      m_ClimberSubsystem.Climb();
    } else if (RelayJoystick.getRawButton(11)) {
      m_ClimberSubsystem.Descend();
    } else {
      m_ClimberSubsystem.Stop();
    }
  }
}
