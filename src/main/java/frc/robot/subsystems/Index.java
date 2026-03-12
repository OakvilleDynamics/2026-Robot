package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants.MechanismConstants.IndexerConstants;

public class Index extends SubsystemBase {
  private final SparkFlex IndexMotor =
      new SparkFlex(MechanismConstants.kIndexMotor, SparkLowLevel.MotorType.kBrushless);

  public Index() {
    IndexMotor.setInverted(IndexerConstants.kIndex_Inverted);

    System.out.println("Index subsystem initialized.");
  }

  public void IndexMove() {
    IndexMotor.set(IndexerConstants.kIndex_Speed);
  }

  public void IndexReverse() {
    IndexMotor.set(-IndexerConstants.kIndex_Speed);
  }

  public void IndexStop() {
    IndexMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
