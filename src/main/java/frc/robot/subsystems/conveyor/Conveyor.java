// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.conveyor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.util.PhoenixUtil;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Conveyor extends SubsystemBase {
  TalonFX conveyor = new TalonFX(ConveyorConstants.CONVEYOR_MOTOR_PORT);
  TalonFXConfiguration config = new TalonFXConfiguration();

  /** Creates a new Conveyor. */
  public Conveyor() {
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.MotorOutput.PeakForwardDutyCycle = 1.0;
    config.MotorOutput.PeakReverseDutyCycle = -1.0;


    PhoenixUtil.tryUntilOk(5, ()->conveyor.getConfigurator().apply(config));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Logging current draw
    //  Logger.recordOutput(getName() + "/supplyCurrent", conveyor.getSupplyCurrent().getValueAsDouble());
    //  Logger.recordOutput(getName() + "/statorCurrent", conveyor.getStatorCurrent().getValueAsDouble());
  }

  @AutoLogOutput(key = "Conveyor/velocity")
  public double getLeftFunnelVelocity() {
    return conveyor.getVelocity().getValueAsDouble();
  }

  @AutoLogOutput(key = "Conveyor/dutyCycle")
  public double setLeftFunnelDutyCycle(double dc) {
    conveyor.set(dc);
    return dc;
  }

  public Command setConveyorCommand(double speed) {
    return run(() -> {
      setLeftFunnelDutyCycle(speed);
    });
  }
}