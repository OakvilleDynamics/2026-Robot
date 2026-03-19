package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Climber extends SubsystemBase {
  private final TalonSRX ClimberMotor = new TalonSRX(MechanismConstants.ClimberMotor);

  public Climber() {
    // Constructor code here, if needed
    ClimberMotor.setInverted(MechanismConstants.ClimberMotor_Inverted);
  }

  /**
   * Activates the climber motor to move in the forward direction at a predefined speed.
   */
  public void Climb() {
    ClimberMotor.set(TalonSRXControlMode.PercentOutput, MechanismConstants.Climber_Speed);
  }

  /**
   * Activates the climber motor to move in the backward direction at a predefined speed.
   */
  public void Descend() {
    ClimberMotor.set(TalonSRXControlMode.PercentOutput, -MechanismConstants.Climber_Speed);
  }

  /**
   * Stops the climber motor.
   */
  public void Stop() {
    ClimberMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Climber/ClimberMotorOutput", ClimberMotor.getMotorOutputPercent());
    Logger.recordOutput("Climber/ClimberMotorCurrent", ClimberMotor.getStatorCurrent());
    Logger.recordOutput("Climber/ClimberMotorTemperature", ClimberMotor.getTemperature());
  }
}
