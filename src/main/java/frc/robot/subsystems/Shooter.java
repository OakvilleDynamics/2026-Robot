// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/* Use if using REV
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel; **/
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Shooter extends SubsystemBase {
  /* Use if using REV
  private final SparkFlex ShooterMotor =
      new SparkFlex(MechanismConstants.ShooterMotor, SparkLowLevel.MotorType.kBrushless); */

  private final TalonFX ShooterMotor = new TalonFX(MechanismConstants.ShooterMotor);

  public void Shoot() {
    ShooterMotor.set(MechanismConstants.Shooter_Speed);
  }

  public void StopShoot() {
    ShooterMotor.set(MechanismConstants.Shooter_Speed);
  }
}
