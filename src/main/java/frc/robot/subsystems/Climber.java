package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final Relay Relay1 = new Relay(0);

  /** Runs climber relay forward to extend the climber. */
  public void RelayOn() {
    Relay1.set(Relay.Value.kReverse);
  }

  /** Runs climber relay in reverse to retract the climber. */
  public void RelayBack() {
    Relay1.set(Relay.Value.kForward);
  }

  /** Sets climber relay off */
  public void RelayOff() {
    Relay1.set(Relay.Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Relay1", Relay1.get().toString());
  }
}
