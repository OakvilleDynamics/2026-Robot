package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends Command {

  private final Climber m_RelaySubsystem;

  private final Joystick RelayJoystick = new Joystick(OperatorConstants.COPILOT_CONTROLLER);

  public ClimberCommand(Climber subsystem) {
    m_RelaySubsystem = subsystem;
    addRequirements(m_RelaySubsystem);
  }

  public void execute() {
    if (RelayJoystick.getRawButton(10)) {
      m_RelaySubsystem.RelayOn();
    } else {
      m_RelaySubsystem.RelayOff();
    }
  }
}

