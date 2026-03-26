package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants.MechanismConstants.IndexerConstants;

public class Index extends SubsystemBase {
  private SparkFlex m_index;
  private SparkFlexConfig c_indexConfig;

  public Index() {
    System.out.println("[Index] Initializing Index Subsystem...");

    // Initialize the index motor and its configuration
    m_index = new SparkFlex(MechanismConstants.INDEX_MOTOR, SparkLowLevel.MotorType.kBrushless);
    c_indexConfig = new SparkFlexConfig();

    // Set inversion for the index motor
    c_indexConfig.inverted(IndexerConstants.INVERTED);

    // Apply configuration to the motor, resetting to safe parameters and persisting the new
    // parameters
    m_index.configure(
        c_indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    System.out.println("[Index] Index Subsystem Initialized!");
  }

  /** Runs the indexer to the shooter */
  public void IndexMove() {
    m_index.set(IndexerConstants.SPEED);
  }

  /** Runs the indexer in reverse to clear jams */
  public void IndexReverse() {
    m_index.set(-IndexerConstants.SPEED);
  }

  /** Stops the indexer motor */
  public void IndexStop() {
    m_index.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
