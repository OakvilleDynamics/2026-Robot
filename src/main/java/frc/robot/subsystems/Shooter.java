// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants.MechanismConstants.ShooterConstants;

public class Shooter extends SubsystemBase {
  // Shooter motor
  private final TalonFX shooterMotor = new TalonFX(MechanismConstants.kShooterMotor);
  ;
  private final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
  private double rpmSetpoint;

  // Create a new Shooter subsystem.
  public Shooter() {
    // Configure the shooter motor with the PID constants from Constants
    shooterConfig.Slot0.kS = ShooterConstants.kShooter_kS;
    shooterConfig.Slot0.kV = ShooterConstants.kShooter_kV;
    shooterConfig.Slot0.kP = ShooterConstants.kShooter_kP;
    shooterConfig.Slot0.kI = ShooterConstants.kShooter_kI;
    shooterConfig.Slot0.kD = ShooterConstants.kShooter_kD;

    // Set current limits to 40amps to prevent damage to the motor and ensure consistent
    // performance. Adjust as needed based on the specific motor and mechanism.
    shooterConfig
        .TorqueCurrent
        .withPeakForwardTorqueCurrent(Amps.of(40))
        .withPeakReverseTorqueCurrent(Amps.of(40));

    // Apply the configurations to the motor
    shooterMotor.getConfigurator().apply(shooterConfig);

    // Display shooter PID constants on SmartDashboard for tuning
    SmartDashboard.putNumber("Shooter kS", ShooterConstants.kShooter_kS);
    SmartDashboard.putNumber("Shooter kV", ShooterConstants.kShooter_kV);
    SmartDashboard.putNumber("Shooter kP", ShooterConstants.kShooter_kP);
    SmartDashboard.putNumber("Shooter kI", ShooterConstants.kShooter_kI);
    SmartDashboard.putNumber("Shooter kD", ShooterConstants.kShooter_kD);

    rpmSetpoint = 0;
    shooterConfig.MotorOutput.Inverted = ShooterConstants.kShooterMotor_Inverted;

    System.out.println("Shooter subsystem initialized.");
  }

  /** Runs the shooter to a specified control request */
  private void runControl(double rpm) {
    var request = new VelocityVoltage(rpm);

    shooterMotor.setControl(request.withVelocity(8));
  }

  /**
   * Starts the motor priming process. This should be called when the robot is enabled and the
   * shooter needs to be primed before shooting. The motor will run at a lower speed to get up to
   * speed before shooting.
   */
  public void ShootStart() {
    shooterMotor.set(ShooterConstants.kShooter_Speed2);
  }

  /**
   * Sets the shooter motor to the specified speed. This should be called when the shooter needs to
   * be activated, such as when the robot is enabled and the shooting action is initiated.
   */
  public void Shoot() {
    shooterMotor.set(ShooterConstants.kShooter_Speed1);
  }

  /**
   * Stops the shooter motor. This should be called when the shooter is no longer needed, such as
   * when the robot is disabled or when the shooting action is complete.
   */
  public void StopShoot() {
    shooterMotor.stopMotor();
  }

  /** Runs shooter at specific RPM. */
  public Command runAtRpm(double rpm) {
    return run(() -> runControl(rpm));
  }

  /** Runs shooter at setpoint RPM. */
  public Command runAtSetpoint() {
    return run(() -> runControl(rpmSetpoint));
  }

  /** Manually specify the setpoint of the shooter */
  public void setRpmSetpoint(double rpm) {
    this.rpmSetpoint = rpm;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update SmartDashboard with the current shooter velocity for monitoring and tuning purposes
    SmartDashboard.putNumber(
        "Shooter Velocity (RPM)", shooterMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Setpoint (RPM)", rpmSetpoint);
    SmartDashboard.putNumber(
        "Shooter Output Voltage (V)", shooterMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter Output Current (A)", shooterMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Speed", shooterMotor.get());
  }
}
