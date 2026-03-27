// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants.MechanismConstants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final TalonFX m_shooterMotor;
  private final TalonFXConfiguration c_shooterConfig;
  private final VelocityVoltage m_request;

  public Shooter() {
    System.out.println("[Shooter] Initializing Shooter Subsystem...");

    // Initialize the shooter motor and its configuration
    m_shooterMotor = new TalonFX(MechanismConstants.SHOOTER_MOTOR);
    c_shooterConfig = new TalonFXConfiguration();

    // Factory default to ensure a known starting point
    m_shooterMotor.getConfigurator().apply(c_shooterConfig);

    // Set motor inversion
    c_shooterConfig.MotorOutput.Inverted = ShooterConstants.INVERTED;

    // Set PID coefficients for closed-loop control
    c_shooterConfig.Slot0.kP = ShooterConstants.P;
    c_shooterConfig.Slot0.kI = ShooterConstants.I;
    c_shooterConfig.Slot0.kD = ShooterConstants.D;
    c_shooterConfig.Slot0.kS = ShooterConstants.S;
    c_shooterConfig.Slot0.kV = ShooterConstants.V;

    // Apply the configuration to the motor controller
    m_shooterMotor.getConfigurator().apply(c_shooterConfig);

    m_request = new VelocityVoltage(0).withSlot(0);

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

  /** Run shooter to a specified velocity */
  public void RunShooterVelocity() {
    m_shooterMotor.setControl(m_request.withVelocity(ShooterConstants.SetVelocity));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
  }
}
