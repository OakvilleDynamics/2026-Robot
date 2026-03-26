// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants.MechanismConstants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final TalonFX m_shooterMotor;
  private final TalonFXConfiguration m_shooterConfig;

  public Shooter() {
    System.out.println("[Shooter] Initializing Shooter Subsystem...");

    // Initialize the shooter motor and its configuration
    m_shooterMotor = new TalonFX(MechanismConstants.SHOOTER_MOTOR);
    m_shooterConfig = new TalonFXConfiguration();

    // Factory default to ensure a known starting point
    m_shooterMotor.getConfigurator().apply(m_shooterConfig);

    // Set motor inversion
    m_shooterConfig.MotorOutput.Inverted = ShooterConstants.INVERTED;

    //// Set PID coefficients for closed-loop control
    // m_shooterConfig.Slot0.kP = ShooterConstants.P;
    // m_shooterConfig.Slot0.kI = ShooterConstants.I;
    // m_shooterConfig.Slot0.kD = ShooterConstants.D;

    // Apply the configuration to the motor controller
    m_shooterMotor.getConfigurator().apply(m_shooterConfig);

    // Shooter motor telemetry
    Logger.recordOutput("Shooter/Output", m_shooterMotor.get());
    Logger.recordOutput(
        "Shooter/Current", m_shooterMotor.getSupplyCurrent().getValueAsDouble(), Units.Amps);
    Logger.recordOutput(
        "Shooter/Temperature", m_shooterMotor.getDeviceTemp().getValueAsDouble(), Units.Celsius);
    Logger.recordOutput(
        "Shooter/Velocity",
        m_shooterMotor.getVelocity().getValueAsDouble(),
        Units.RotationsPerSecond);

    System.out.println("[Shooter] Shooter Subsystem Initialized!");
  }

  /** Spins up the shooter motor to a slower speed than final speed */
  public void SpinUpShooter() {
    m_shooterMotor.set(ShooterConstants.SPEED_SPIN_UP);
  }

  /** Shoots the fuel at final speed */
  public void Shoot() {
    m_shooterMotor.set(ShooterConstants.SPEED_MAIN);
  }

  /** Stops the shooter motor */
  public void StopShoot() {
    m_shooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
