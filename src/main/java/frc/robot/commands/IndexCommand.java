package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Index;

public class IndexCommand extends Command {
  private final Index m_IndexSubsystem;

  private final Joystick IndexJoystick = new Joystick(OperatorConstants.kCOPILOT_CONTROLLER);

  public IndexCommand(Index subsystem) {
    m_IndexSubsystem = subsystem;
    addRequirements(m_IndexSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if (IndexJoystick.getRawButton(5)) {
      m_IndexSubsystem.IndexMove();
    } else if (IndexJoystick.getRawButton(6)) {
      m_IndexSubsystem.IndexReverse();
    } else {
      m_IndexSubsystem.IndexStop();
    }
  }
}
