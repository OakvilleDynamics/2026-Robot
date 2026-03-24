// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  // Create a new Shooter subsystem.
  public Shooter() {
    // Configure the shooter motor with the PID constants from Constants
    shooterConfig.Slot0.kS = ShooterConstants.kShooter_kS;
    shooterConfig.Slot0.kV = ShooterConstants.kShooter_kV;
    shooterConfig.Slot0.kP = ShooterConstants.kShooter_kP;
    shooterConfig.Slot0.kI = ShooterConstants.kShooter_kI;
    shooterConfig.Slot0.kD = ShooterConstants.kShooter_kD;

  // private final TalonFXConfiguration ShooterConfig = new TalonFXConfiguration();

  public Shooter() {
    System.out.println("[Shooter] Initializing Shooter Subsystem...");
    // Constructor code here, if needed
    // ShooterConfig.MotorOutput.Inverted = MechanismConstants.ShooterMotor_Inverted;
    // ShooterConfig.Slot0.kP = MechanismConstants.Shooter_kP;
    // ShooterConfig.Slot0.kI = MechanismConstants.Shooter_kI;
    // ShooterConfig.Slot0.kD = MechanismConstants.Shooter_kD;
    // ShooterConfig.Slot0.kF = MechanismConstants.Shooter_kF;
    // ShooterMotor.getConfigurator().apply(ShooterConfig);
    System.out.println("[Shooter] Shooter Subsystem Initialized!");
  }

  /** Spins up the shooter motor to a slower speed than final speed */
  public void SpinUpShooter() {
    ShooterMotor.set(MechanismConstants.Shooter_Speed2);
  }

  /** Shoots the fuel at final speed */
  public void Shoot() {
    shooterMotor.set(ShooterConstants.kShooter_Speed1);
  }

  /** Stops the shooter motor */
  public void StopShoot() {
    shooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // Shooter motor telemetry
    Logger.recordOutput("Shooter/Output", ShooterMotor.get());
    Logger.recordOutput(
        "Shooter/Current", ShooterMotor.getSupplyCurrent().getValueAsDouble(), Units.Amps);
    Logger.recordOutput(
        "Shooter/Temperature", ShooterMotor.getDeviceTemp().getValueAsDouble(), Units.Celsius);
    Logger.recordOutput(
        "Shooter/Velocity",
        ShooterMotor.getVelocity().getValueAsDouble(),
        Units.RotationsPerSecond);
  }
}
