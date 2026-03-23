package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final TalonSRX ClimberMotor = new TalonSRX(MechanismConstants.ClimberMotor);
  private final TalonSRXConfiguration config = new TalonSRXConfiguration();

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

  /**
   * Get climber motor output in a percentage. This can be negative.
   *
   * @return Percentage value between -1 and 1
   */
  @AutoLogOutput(key = "Climber/Output")
  public double getMotorOutput() {
    return ClimberMotor.getMotorOutputPercent();
  }

  /**
   * Get climber motor controller current output in Amps.
   *
   * @return Amperage of motor
   */
  @AutoLogOutput(key = "Climber/Motor Current")
  public double getMotorCurrent() {
    return ClimberMotor.getSupplyCurrent();
  }

  /**
   * Get climber motor controller temperature in Celsius.
   *
   * @return Motor controller temperature
   */
  @AutoLogOutput(key = "Climber/Motor Temperature")
  public double getMotorTemp() {
    return ClimberMotor.getTemperature();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Climber/Output", ClimberMotor.getMotorOutputPercent());
    Logger.recordOutput("Climber/Current", ClimberMotor.getStatorCurrent());
    Logger.recordOutput("Climber/Motor Temperature", ClimberMotor.getTemperature());
  }
}
