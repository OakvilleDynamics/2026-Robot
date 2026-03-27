package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants.MechanismConstants.IntakeConstants;
import frc.robot.Constants.MechanismConstants.IntakeConstants.HingeConstants;
import org.littletonrobotics.junction.Logger;

// Very basic intake process, changes to be made accordingly
public class Intake extends SubsystemBase {
  private SparkFlex m_intakeRoller, m_intakeHinge;
  private SparkFlexConfig c_intakeHingeConfig, c_intakeRollerConfig;
  private RelativeEncoder re_intakeHingeEncoder;
  private RelativeEncoder ex_intakeHingeEncoder;
  private SparkClosedLoopController ctrl_intakeHingeMotor;

  public Intake() {
    System.out.println("[Intake] Initializing Intake Subsystem...");

    // Initialize the intake roller and hinge motors
    m_intakeRoller =
        new SparkFlex(MechanismConstants.INTAKE_ROLLER_MOTOR, SparkLowLevel.MotorType.kBrushless);
    m_intakeHinge =
        new SparkFlex(MechanismConstants.INTAKE_HINGE_MOTOR, SparkLowLevel.MotorType.kBrushless);

    // Initialize the configurations for the intake roller and hinge motors
    c_intakeRollerConfig = new SparkFlexConfig();
    c_intakeHingeConfig = new SparkFlexConfig();

    // Initialize the encoders for the intake hinge and configure them
    re_intakeHingeEncoder = m_intakeHinge.getEncoder();
    ex_intakeHingeEncoder = m_intakeHinge.getExternalEncoder();

    // Set inversion for the intake roller and hinge motors, and apply configurations
    c_intakeRollerConfig.inverted(IntakeConstants.ROLLER_INVERTED);
    c_intakeHingeConfig.inverted(IntakeConstants.HINGE_INVERTED);

    // Configure the intake hinge motor for closed-loop control
    c_intakeHingeConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .pid(HingeConstants.P, HingeConstants.I, HingeConstants.D)
        .outputRange(-IntakeConstants.HINGE_SPEED_RAISE, IntakeConstants.HINGE_SPEED_RAISE)
        .feedForward
        // kV is now in Volts, so we multiply by the nominal voltage (12V)
        .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

    c_intakeHingeConfig.absoluteEncoder.inverted(true).zeroOffset(0).zeroCentered(false);

    // Set idle modes and current limits for the motors
    c_intakeHingeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);

    // Apply configurations to the motors, resetting to safe parameters and persisting the new
    // parameters
    m_intakeRoller.configure(
        c_intakeRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_intakeHinge.configure(
        c_intakeHingeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    ctrl_intakeHingeMotor = m_intakeHinge.getClosedLoopController();

    System.out.println("[Intake] Intake Subsystem Initialized!");
  }

  public void IntakeFuel() {
    m_intakeRoller.set(IntakeConstants.ROLLER_SPEED);
  }

  /** Expels fuel from the intake. */
  public void IntakeSpit() {
    m_intakeRoller.set(-IntakeConstants.ROLLER_SPEED);
  }

  /** Sets the hinge of the intake mechanism to move up. */
  public void IntakeUp() {
    m_intakeHinge.set(IntakeConstants.HINGE_SPEED);
  }

  /** Sets the hinge of the intake mechanism to move down. */
  public void IntakeDown() {
    m_intakeHinge.set(-IntakeConstants.HINGE_SPEED);
  }

  /** Stops the hinge from moving */
  public void IntakeHingeStop() {
    m_intakeHinge.set(0);
  }

  /** Stops the roller from moving */
  public void IntakeRollerStop() {
    m_intakeRoller.set(0);
  }

  /** Set the intake hinge to run to a position and hold it there */
  public Command setIntakeHingePosition(double pos) {
    return run(() -> ctrl_intakeHingeMotor.setSetpoint(pos, ControlType.kPosition));
  }

  /** Resets the intake hinge encoder to 0. */
  public void resetIntakeHingeEncoder() {
    ex_intakeHingeEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // IntakeRoller telemetry
    Logger.recordOutput("Intake/Roller/Output", m_intakeRoller.get());
    Logger.recordOutput("Intake/Roller/Current", m_intakeRoller.getOutputCurrent(), Units.Amps);
    Logger.recordOutput(
        "Intake/Roller/Temperature", m_intakeRoller.getMotorTemperature(), Units.Celsius);

    // IntakeHinge telemetry
    Logger.recordOutput("Intake/Hinge/Output", m_intakeHinge.get());
    Logger.recordOutput("Intake/Hinge/Current", m_intakeHinge.getOutputCurrent(), Units.Amps);
    Logger.recordOutput(
        "Intake/Hinge/Temperature", m_intakeHinge.getMotorTemperature(), Units.Celsius);
    Logger.recordOutput(
        "Intake/Hinge/Internal Encoder Position", m_intakeHinge.getEncoder().getPosition());
    Logger.recordOutput(
        "Intake/Hinge/External Encoder Position",
        ex_intakeHingeEncoder.getPosition(),
        Units.Rotations);
    Logger.recordOutput(
        "Intake/Hinge/External Encoder Velocity",
        ex_intakeHingeEncoder.getVelocity(),
        Units.RotationsPerSecond);
    Logger.recordOutput(
        "Intake/Hinge/Internal Encoder Velocity",
        re_intakeHingeEncoder.getVelocity(),
        Units.RotationsPerSecond);
  }
}
