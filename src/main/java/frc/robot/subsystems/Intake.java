package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;
import org.littletonrobotics.junction.Logger;

// Very basic intake process, changes to be made accordingly

public class Intake extends SubsystemBase {
  private final SparkFlex IntakeRoller =
      new SparkFlex(MechanismConstants.IntakeRoller, SparkLowLevel.MotorType.kBrushless);

  public final SparkFlex IntakeHinge =
      new SparkFlex(MechanismConstants.IntakeHinge, SparkLowLevel.MotorType.kBrushless);

  public Intake() {
    System.out.println("[Intake] Initializing Intake Subsystem...");
    IntakeRoller.setInverted(MechanismConstants.IntakeRoller_Inverted);
    IntakeHinge.setInverted(MechanismConstants.IntakeHinge_Inverted);
    System.out.println("[Intake] Intake Subsystem Initialized!");
  }

  public void IntakeFuel() {
    IntakeRoller.set(MechanismConstants.Intake_Speed);
  }

  public void IntakeSpit() {
    IntakeRoller.set(-MechanismConstants.Intake_Speed);
  }

  public void IntakeUp() {
    IntakeHinge.set(MechanismConstants.Intake_Hinge_Speed);
  }

  public void IntakeDown() {
    IntakeHinge.set(-MechanismConstants.Intake_Hinge_Speed);
  }

  public void IntakeStop() {
    IntakeRoller.set(0);
    IntakeHinge.set(0);
  }

  @Override
  public void periodic() {
    // IntakeRoller telemetry
    Logger.recordOutput("Intake/Roller/Motor Output", IntakeRoller.get());
    Logger.recordOutput("Intake/Roller/Current", IntakeRoller.getOutputCurrent(), Units.Amps);
    Logger.recordOutput("Intake/Roller/Temperature", IntakeRoller.getMotorTemperature(), Units.Celsius);

    // IntakeHinge telemetry
    Logger.recordOutput("Intake/Hinge/Motor Output", IntakeHinge.get());
    Logger.recordOutput("Intake/Hinge/Current", IntakeHinge.getOutputCurrent(), Units.Amps);
    Logger.recordOutput("Intake/Hinge/Temperature", IntakeHinge.getMotorTemperature(), Units.Celsius);
    Logger.recordOutput("Intake/Hinge/Encoder", IntakeHinge.getEncoder().getPosition());
    Logger.recordOutput("Intake/Hinge/Velocity", IntakeHinge.getEncoder().getVelocity(), Units.RPM);
  }
}
