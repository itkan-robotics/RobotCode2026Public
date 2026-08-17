// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.transfer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PhoenixUtil;

import static frc.robot.Constants.TransferConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Transfer extends SubsystemBase {
  public boolean feeding = false;
  public boolean hasPreloaded = false;
  public Timer feedTimer = new Timer();
  public double feedPower = 0;
  TalonFX rightTransfer = new TalonFX(RIGHT_TRANSFER_MOTOR_PORT);
  TalonFX leftTransfer = new TalonFX(LEFT_TRANSFER_MOTOR_PORT);
  TalonFXConfiguration config = new TalonFXConfiguration();

  /** Creates a new Transfer. */
  public Transfer() {
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 90;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 45;
    PhoenixUtil.tryUntilOk(5, () -> rightTransfer.getConfigurator().apply(config));
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    PhoenixUtil.tryUntilOk(5,
    () -> leftTransfer.getConfigurator().apply(config));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Logging current draw
    // Logger.recordOutput(getName() + "/supplyCurrent", rightTransfer.getSupplyCurrent().getValueAsDouble());
    // Logger.recordOutput(getName() + "/statorCurrent", rightTransfer.getStatorCurrent().getValueAsDouble());
  }

  @AutoLogOutput(key = "Transfer/transferVelocity")
  public double getTransferVelocity() {
    return rightTransfer.getVelocity().getValueAsDouble();
  }

  @AutoLogOutput(key = "Transfer/transferTargetDutyCycle")
  public double setTransferDutyCycle(double dc) {
    rightTransfer.set(dc);
    leftTransfer.set(dc);
    return dc;
  }

  public Command setTransferCommand(double speed) {
    return run(() -> {
      setTransferDutyCycle(speed);
    });
  }

  public Command preloadTransferCommand() {
    return run(()-> {
      if (!hasPreloaded) setTransferDutyCycle(0.4);
      hasPreloaded = true;
    });
  }

  public double getStatorCurrent() {
    return leftTransfer.getStatorCurrent().getValueAsDouble();
  }

  public double getFeedPower() {
    return feedPower;
  }

  public void setFeedPower(double feedPower) {
    this.feedPower = feedPower;
  }
}
