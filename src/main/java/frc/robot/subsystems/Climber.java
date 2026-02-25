package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final Relay Relay1 = new Relay(0);

  public void RelayOn() {
    Relay1.set(Relay.Value.kForward);
  }

  public void RelayOff() {
    Relay1.set(Relay.Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Relay1", Relay1.get().toString());
  }
}