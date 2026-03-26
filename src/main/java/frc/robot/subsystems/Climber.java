package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants.MechanismConstants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final TalonSRX m_climber;

  public Climber() {
    System.out.println("[Climber] Initializing Climber Subsystem...");

    // Initialize the climber motor
    m_climber = new TalonSRX(MechanismConstants.CLIMBER_MOTOR);

    // Set motor inversion
    m_climber.setInverted(ClimberConstants.INVERTED);

    System.out.println("[Climber] Climber Subsystem Initialized!");
  }

  /** Activates the climber motor to move in the forward direction at a predefined speed. */
  public void Climb() {
    m_climber.set(TalonSRXControlMode.PercentOutput, ClimberConstants.SPEED);
  }

  /** Activates the climber motor to move in the backward direction at a predefined speed. */
  public void Descend() {
    m_climber.set(TalonSRXControlMode.PercentOutput, -ClimberConstants.SPEED);
  }

  /** Stops the climber motor. */
  public void Stop() {
    m_climber.set(TalonSRXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This subsystem is so simple that we don't really need to log anything here.
  }
}
