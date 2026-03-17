package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants.MechanismConstants.ClimberConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class Climber extends SubsystemBase {
  private final TalonSRX ClimberMotor = new TalonSRX(MechanismConstants.ClimberMotor);
  private final TalonSRXConfiguration config = new TalonSRXConfiguration();

  public Climber() {
    // Constructor code here, if needed
    ClimberMotor.setInverted(ClimberConstants.ClimberMotor_Inverted);

    // Apply motor configuration
    ClimberMotor.configAllSettings(config);
  }

  /** Runs climber motor forward to extend the climber. */
  public void ClimbUp() {
    ClimberMotor.set(TalonSRXControlMode.PercentOutput, ClimberConstants.Climber_Speed);
  }

  /** Runs climber motor in reverse to retract the climber. */
  public void LowerDown() {
    ClimberMotor.set(TalonSRXControlMode.PercentOutput, -ClimberConstants.Climber_Speed);
  }

  /** Sets climber motor off */
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
    SmartDashboard.putNumber("Climber Motor Output", getMotorOutput());
    SmartDashboard.putNumber("Climber Motor Current", getMotorCurrent());
    SmartDashboard.putNumber("Climber Motor Temperature", getMotorTemp());
  }
}
