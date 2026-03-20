package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;
import org.littletonrobotics.junction.Logger;

// Very basic intake process, changes to be made accordingly

public class Intake extends SubsystemBase {
  private final SparkFlex IntakeMotor =
      new SparkFlex(MechanismConstants.IntakeMotor, SparkLowLevel.MotorType.kBrushless);

  public final SparkFlex IntakeHinge =
      new SparkFlex(MechanismConstants.IntakeHinge, SparkLowLevel.MotorType.kBrushless);

  public Intake() {
    IntakeMotor.setInverted(MechanismConstants.IntakeMotor_Inverted);
    IntakeHinge.setInverted(MechanismConstants.IntakeHinge_Inverted);
  }

  public void IntakeFuel() {
    IntakeMotor.set(MechanismConstants.Intake_Speed);
  }

  public void IntakeSpit() {
    IntakeMotor.set(-MechanismConstants.Intake_Speed);
  }

  public void IntakeUp() {
    IntakeHinge.set(MechanismConstants.Intake_Hinge_Speed);
  }

  public void IntakeDown() {
    IntakeHinge.set(-MechanismConstants.Intake_Hinge_Speed);
  }

  public void IntakeStop() {
    IntakeMotor.set(0);
    IntakeHinge.set(0);
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
