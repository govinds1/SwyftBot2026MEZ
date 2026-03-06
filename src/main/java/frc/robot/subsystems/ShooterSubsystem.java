// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
  private final SparkFlex m_shooterMotor;
  private final SparkFlex m_shooterMotorFollower;
  private final SparkFlex m_feederMotor;

  private final RelativeEncoder m_encoder;

  private final SparkClosedLoopController m_closedLoopController;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public ShooterSubsystem() {
    m_shooterMotor = new SparkFlex(ShooterConstants.kRightShooterMotorCanId, MotorType.kBrushless);
    m_shooterMotorFollower = new SparkFlex(ShooterConstants.kLeftShooterMotorCanId, MotorType.kBrushless);
    m_feederMotor = new SparkFlex(ShooterConstants.kFeederMotorCanId, MotorType.kBrushless);

    m_encoder = m_shooterMotor.getEncoder();
    m_closedLoopController = m_shooterMotor.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_shooterMotor.configure(Configs.Shooter.shooterMotorConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    SparkBaseConfig followerConfig = Configs.Shooter.shooterMotorConfig;
    followerConfig.follow(m_shooterMotor, true); // Set to follow main motor, and invert.
    m_shooterMotorFollower.configure(followerConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    m_feederMotor.configure(Configs.Shooter.feederMotorConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Subsytems/Shooter/Flywheel/RPMCurrent", m_encoder.getVelocity());
  }

  /**
   * Run flywheel to speed.
   * 
   * @param desiredRPM RPM to set as motor velocity setpoint.
   */
  public void runShooterRPM() {  
    m_closedLoopController.setSetpoint(
        ShooterConstants.kShootingRPM, 
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0
    );
  }

  /**
   * Run flywheel on open loop control.
   * 
   * @param power Open loop power (-1 to 1)
   */
  public void runShooterOpenLoop() {
    m_shooterMotor.set(ShooterConstants.kShooterPower);
  }

  public Command runShooterRPMCommand() {
    return Commands.sequence(
      Commands.runOnce(() -> runShooterRPM(), this),
      Commands.runOnce(() -> runFeeder(ShooterConstants.kFeederPower), this).onlyIf(this::isAtSpeed)
    );
  }

  public Command runShooterOpenLoopCommand() {
    return Commands.sequence(
      Commands.runOnce(() -> runShooterOpenLoop(), this),
      new WaitCommand(ShooterConstants.kShooterSpinUpTime),
      Commands.runOnce(() -> runFeeder(ShooterConstants.kFeederPower), this)
    );
  }

  /**
   * Stop flywheel.
   */
  public void stopShooter() {
    m_shooterMotor.stopMotor();
  }

    /**
   * Run feeder on open loop control.
   * 
   * @param desiredRPM RPM to set as motor velocity setpoint.
   */
  public void runFeeder(double power) {
    //m_desiredRPM = desiredRPM;
    m_feederMotor.set(power);
  }

  /**
   * Stop feeder.
   */
  public void stopFeeder() {
    m_feederMotor.stopMotor();
  }

  public void stop() {
    stopShooter();
    stopFeeder();
  }

  /**
   * Check if flywheel is at speed.
   * 
   * @returns true if motor has reached the velocity setpoint.
   */
  public boolean isAtSpeed() {
    return Math.abs(m_encoder.getVelocity() - ShooterConstants.kShootingRPM) < ShooterConstants.kRPMTolerance;
  }

  /**
   * Read flywheel speed.
   * 
   * @returns actual flywheel motor RPM.
   */
  public double getActualSpeedRPM() {
    return m_encoder.getVelocity();
  }
}