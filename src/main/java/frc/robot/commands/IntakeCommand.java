package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
  private final Intake m_IntakeSubsystem;

  private final Joystick IntakeJoystick = new Joystick(OperatorConstants.COPILOT_CONTROLLER);

  public IntakeCommand(Intake subsystem) {
    m_IntakeSubsystem = subsystem;
    addRequirements(m_IntakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Button 1 is simply a placeholder button, probably will be changed
    if (IntakeJoystick.getRawButton(2)) {
      m_IntakeSubsystem.IntakeFuel();
    } else if (IntakeJoystick.getRawButton(3)) {
      m_IntakeSubsystem.IntakeUp();
    } else if (IntakeJoystick.getRawButton(4)) {
      m_IntakeSubsystem.IntakeUp();
    } else {
      m_IntakeSubsystem.IntakeStop();
    }
  }
}
