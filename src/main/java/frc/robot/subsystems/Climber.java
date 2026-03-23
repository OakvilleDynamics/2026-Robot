package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final TalonSRX ClimberMotor = new TalonSRX(MechanismConstants.ClimberMotor);

  public Climber() {
    // Constructor code here, if needed
    System.out.println("[Climber] Initializing Climber Subsystem...");
    ClimberMotor.setInverted(MechanismConstants.ClimberMotor_Inverted);
    System.out.println("[Climber] Climber Subsystem Initialized!");
  }

  /** Activates the climber motor to move in the forward direction at a predefined speed. */
  public void Climb() {
    ClimberMotor.set(TalonSRXControlMode.PercentOutput, MechanismConstants.Climber_Speed);
  }

  /** Activates the climber motor to move in the backward direction at a predefined speed. */
  public void Descend() {
    ClimberMotor.set(TalonSRXControlMode.PercentOutput, -MechanismConstants.Climber_Speed);
  }

  /** Stops the climber motor. */
  public void Stop() {
    ClimberMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Climber/Output", ClimberMotor.getMotorOutputPercent());
    Logger.recordOutput("Climber/Current", ClimberMotor.getStatorCurrent(), Units.Amps);
    Logger.recordOutput("Climber/Motor Temperature", ClimberMotor.getTemperature(), Units.Celsius);
  }
}
