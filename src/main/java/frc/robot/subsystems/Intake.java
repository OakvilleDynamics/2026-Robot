package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

// Very basic intake process, changes to be made accordingly

public class Intake extends SubsystemBase {
  private final SparkFlex IntakeMotor =
      new SparkFlex(MechanismConstants.IntakeMotor, SparkLowLevel.MotorType.kBrushless);

  public final SparkFlex IntakeHinge =
      new SparkFlex(MechanismConstants.IntakeHinge, SparkLowLevel.MotorType.kBrushless);

  public void IntakeFuel() {
    IntakeMotor.set(MechanismConstants.Intake_Speed);
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
}
