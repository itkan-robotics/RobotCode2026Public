// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
  TalonFX frontRightIntake = new TalonFX(FRONT_INTAKE_RIGHT_MOTOR_PORT);
  TalonFX frontLeftIntake = new TalonFX(FRONT_INTAKE_LEFT_MOTOR_PORT);

  TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
  TalonFX pivot = new TalonFX(INTAKE_PIVOT_MOTOR_PORT);
  TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
  MotionMagicVoltage motionMagic = new MotionMagicVoltage(INTAKE_PIVOT_FF);
  double intakeMaxLimit = 0;
  double intakeRange = Math
      .abs(Constants.IntakeConstants.INTAKE_PIVOT_DOWN - Constants.IntakeConstants.INTAKE_PIVOT_UP);

  /** Creates a new Intake. */
  public Intake() {
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 80;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 60;

    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> frontLeftIntake.getConfigurator().apply(intakeConfig));
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> frontRightIntake.getConfigurator().apply(intakeConfig));

    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 45;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 30;

    Slot0Configs pivotSlot0Configs = pivotConfig.Slot0;
    pivotSlot0Configs.kS = INTAKE_PIVOT_KS;
    pivotSlot0Configs.kV = INTAKE_PIVOT_KV;
    pivotSlot0Configs.kA = INTAKE_PIVOT_KA;
    pivotSlot0Configs.kG = INTAKE_PIVOT_KG;
    pivotSlot0Configs.kP = INTAKE_PIVOT_KP;
    pivotSlot0Configs.kI = INTAKE_PIVOT_KI;
    pivotSlot0Configs.kD = INTAKE_PIVOT_KD;
    MotionMagicConfigs pivotMotionMagicConfigs = pivotConfig.MotionMagic;
    pivotMotionMagicConfigs.MotionMagicCruiseVelocity = INTAKE_PIVOT_VELOCITY;
    pivotMotionMagicConfigs.MotionMagicAcceleration = INTAKE_PIVOT_ACCELERATION;
    pivotMotionMagicConfigs.MotionMagicJerk = INTAKE_PIVOT_JERK;

    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = INTAKE_PIVOT_DOWN;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    pivot.getConfigurator().apply(pivotConfig);
  }

  @Override
  public void periodic() {
    if (pivot.getStatorCurrent().getValueAsDouble() > 15
        && Math.abs(pivot.getVelocity().getValueAsDouble()) <= 0.1) {
      setIntakePosition(pivot.getPosition().getValueAsDouble());
    }
  }
  @AutoLogOutput(key = "Intake/Position")
  public double getIntakePosition() {
    return pivot.getPosition().getValueAsDouble();
  }

  @AutoLogOutput(key = "Intake/velocity")
  public double getIntakeVelocity() {
    return frontRightIntake.isConnected() ? frontRightIntake.getVelocity().getValueAsDouble()
        : frontLeftIntake.getVelocity().getValueAsDouble();
  }

  @AutoLogOutput(key = "Intake/targetDutyCycle")
  public double setIntakeDutyCycle(double dc) {
    frontRightIntake.set(dc);
    frontLeftIntake.set(dc);
    return dc;
  }

  @AutoLogOutput(key = "Intake/targetPos")
  public double setIntakePosition(double pos) {
    pivot.setControl(motionMagic.withPosition(pos));
    return pos;
  }

  public Command intakeDefaultCommand() {
    return run(() -> {
      setIntakeDutyCycle(0);
    });
  }

  public Command setIntakeCommand(double speed) {
    return run(() -> {
      setIntakeDutyCycle(speed);
    });
  }

  public Command setIntakePosCommand(double pos) {
    return runOnce(() -> {
      setIntakePosition(pos);
    });
  }

  public boolean withinTolerance() {
    return Math.abs(pivot.getPosition().getValueAsDouble() - motionMagic.Position) < INTAKE_PIVOT_TOLERANCE;
  }

  public Command deployAndIntakeCommand() {
    return Commands.sequence(
        setIntakePosCommand(INTAKE_PIVOT_DOWN),
        Commands.waitUntil(this::withinTolerance).withTimeout(0.5),
        setIntakeCommand(1));
  }

  public Command deployAndOutakeCommand(double speed) {
    return Commands.sequence(
        setIntakePosCommand(INTAKE_PIVOT_DOWN),
        Commands.waitUntil(this::withinTolerance),
        setIntakeCommand(speed));
  }

  public Command intakeDeployCommand() {
    return runOnce(() -> {
      setIntakePosition(INTAKE_PIVOT_DOWN);
    });
  }

  public Command intakeHomeCommand() {
    return runOnce(() -> {
      setIntakePosition(INTAKE_PIVOT_UP);
    });
  }

  public void coast() {
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    pivot.getConfigurator().apply(pivotConfig);
  }

  public Command coastCommand() {
    return runOnce(this::coast);
  }
  

  public void brake() {
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivot.getConfigurator().apply(pivotConfig);
  }

  public Command brakeCommand() {
    return runOnce(this::brake);
  }

  public void zero() {
    double p = pivot.getRotorPosition().getValueAsDouble();
    intakeMaxLimit = p;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = p;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = p - intakeRange;
    pivot.getConfigurator().apply(pivotConfig);
  }

  public Command zeroCommand() {
    return runOnce(this::zero);
  }
}