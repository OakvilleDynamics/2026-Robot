package frc.robot.subsystems;

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

  public void RelayOn() {
    ClimberMotor.set(TalonSRXControlMode.PercentOutput, MechanismConstants.Climber_Speed);
  }

  public void RelayBack() {
    ClimberMotor.set(TalonSRXControlMode.PercentOutput, -MechanismConstants.Climber_Speed);
  }

  public void RelayOff() {
    ClimberMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Motor Output", ClimberMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Climber Motor Current", ClimberMotor.getStatorCurrent());
    SmartDashboard.putNumber("Climber Motor Temperature", ClimberMotor.getTemperature());
  }
}
