package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Index extends SubsystemBase {
  private final SparkFlex IndexMotor =
      new SparkFlex(MechanismConstants.IndexMotor, SparkLowLevel.MotorType.kBrushless);

  public void IndexMove() {
    IndexMotor.set(MechanismConstants.Index_Speed);
  }

  public void IndexReverse() {
    IndexMotor.set(-MechanismConstants.Index_Speed);
  }

  public void IndexStop() {
    IndexMotor.set(0);
  }
}
