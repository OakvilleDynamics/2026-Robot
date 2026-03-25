package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants.MechanismConstants.ClimberConstants;
import org.littletonrobotics.junction.Logger;

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
    // This method will be called once per scheduler run
    Logger.recordOutput("Climber/Output", m_climber.getMotorOutputPercent());
    Logger.recordOutput("Climber/Current", m_climber.getStatorCurrent(), Units.Amps);
    Logger.recordOutput("Climber/Temperature", m_climber.getTemperature(), Units.Celsius);
  }
}
