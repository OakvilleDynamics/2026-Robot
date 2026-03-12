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
import frc.robot.Constants.MechanismConstants.IntakeConstants;

// Very basic intake process, changes to be made accordingly
public class Intake extends SubsystemBase {
  private SparkFlex intakeMotor =
      new SparkFlex(MechanismConstants.kIntakeMotor, MotorType.kBrushless);

  private SparkFlex intakeHinge =
      new SparkFlex(MechanismConstants.kIntakeHinge, MotorType.kBrushless);

  private RelativeEncoder hingeEncoder;

  private final SparkClosedLoopController intakeHingeController;

  public Intake() {
    intakeMotor.configure(
        IntakeConstants.kIntakeMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    intakeHinge.configure(
        IntakeConstants.kIntakeHingeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    hingeEncoder = intakeHinge.getEncoder();

    intakeHingeController = intakeHinge.getClosedLoopController();

    System.out.println("Intake subsystem initialized.");
  }

  /** Runs intake motor to intake fuel. */
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
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Hinge Position", hingeEncoder.getPosition());
    SmartDashboard.putNumber("Intake Hinge Speed", intakeHinge.get());
    SmartDashboard.putNumber("Intake Hinge Current", intakeHinge.getOutputCurrent());
    SmartDashboard.putNumber("Intake Motor Speed", intakeMotor.get());
    SmartDashboard.putNumber("Intake Motor Current", intakeMotor.getOutputCurrent());
  }
}
