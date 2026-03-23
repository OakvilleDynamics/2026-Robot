package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;
import org.littletonrobotics.junction.Logger;

// Very basic intake process, changes to be made accordingly
public class Intake extends SubsystemBase {
  private SparkFlex intakeMotor =
      new SparkFlex(MechanismConstants.kIntakeMotor, MotorType.kBrushless);

  private SparkFlex intakeHinge =
      new SparkFlex(MechanismConstants.kIntakeHinge, MotorType.kBrushless);

  public Intake() {
    System.out.println("[Intake] Initializing Intake Subsystem...");
    IntakeMotor.setInverted(MechanismConstants.IntakeMotor_Inverted);
    IntakeHinge.setInverted(MechanismConstants.IntakeHinge_Inverted);
    System.out.println("[Intake] Intake Subsystem Initialized!");
  }

  public void IntakeFuel() {
    intakeMotor.set(IntakeConstants.kIntake_Speed);
  }

  /** Expells fuel from the intake. */
  public void IntakeSpit() {
    intakeMotor.set(-IntakeConstants.kIntake_Speed);
  }

  /** Sets the hinge of the intake mechanism to move up. */
  public void IntakeUp() {
    intakeHinge.set(IntakeConstants.kIntake_Hinge_Speed);
  }

  /** Sets the hinge of the intake mechanism to move down. */
  public void IntakeDown() {
    intakeHinge.set(-IntakeConstants.kIntake_Hinge_Speed);
  }

  /** Stops intake subsystem entirely. */
  public void IntakeStop() {
    intakeMotor.set(0);
    intakeHinge.set(0);
  }

  @Override
  public void periodic() {
    // IntakeMotor telemetry
    Logger.recordOutput("Intake/Intake/Motor Output", IntakeMotor.get());
    Logger.recordOutput("Intake/Intake/Current", IntakeMotor.getOutputCurrent());
    Logger.recordOutput("Intake/Intake/Temperature", IntakeMotor.getMotorTemperature());

    // IntakeHinge telemetry
    Logger.recordOutput("Intake/Hinge/Motor Output", IntakeHinge.get());
    Logger.recordOutput("Intake/Hinge/Current", IntakeHinge.getOutputCurrent());
    Logger.recordOutput("Intake/Hinge/Temperature", IntakeHinge.getMotorTemperature());
    Logger.recordOutput("Intake/Hinge/Encoder", IntakeHinge.getEncoder().getPosition());
    Logger.recordOutput("Intake/Hinge/Velocity", IntakeHinge.getEncoder().getVelocity());
  }
}
