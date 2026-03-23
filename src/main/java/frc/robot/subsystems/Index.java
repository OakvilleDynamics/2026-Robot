package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;
import org.littletonrobotics.junction.Logger;

public class Index extends SubsystemBase {
  private final SparkFlex IndexMotor =
      new SparkFlex(MechanismConstants.IndexMotor, SparkLowLevel.MotorType.kBrushless);

  public Index() {
    System.out.println("[Index] Initializing Index Subsystem...");
    IndexMotor.setInverted(MechanismConstants.Index_Inverted);
    System.out.println("[Index] Index Subsystem Initialized!");
  }

  /** Runs the indexer to the shooter */
  public void IndexMove() {
    IndexMotor.set(MechanismConstants.Index_Speed);
  }

  /** Runs the indexer in reverse to clear jams */
  public void IndexReverse() {
    IndexMotor.set(-MechanismConstants.Index_Speed);
  }

  /** Stops the indexer motor */
  public void IndexStop() {
    IndexMotor.set(0);
  }

  @Override
  public void periodic() {
    // Index motor telemetry
    Logger.recordOutput("Index/Output", IndexMotor.get());
    Logger.recordOutput("Index/Current", IndexMotor.getOutputCurrent(), Units.Amps);
    Logger.recordOutput("Index/Temperature", IndexMotor.getMotorTemperature(), Units.Celsius);
  }
}
