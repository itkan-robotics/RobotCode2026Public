// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {

  TalonFX rightShooter = new TalonFX(RIGHT_SHOOTER_PORT);
  TalonFX leftShooter = new TalonFX(LEFT_SHOOTER_PORT);
  TalonFXConfiguration config = new TalonFXConfiguration();
  VelocityDutyCycle bangBangOnboard = new VelocityDutyCycle(0);
  VelocityVoltage pidVoltage = new VelocityVoltage(0);

  private LoggedTunableNumber tunableRPS;

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP", 0.4);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD", 0.0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 0.22);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 0.019);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Flywheel/MaxAcceleration", 50.0);
      
  // using equation instead (distance has 11 elements, velocity has 12)
  //double[] distance = new double[] { 2.147, 1.74, 2.073, 2.437, 2.627, 3.016, 3.212, 3.477, 2.785, 3.391 };
  //double[] velocity = new double[] { 39.75, 39.25, 39.6, 41.5, 43.5, 45.7, 46.5, 48.25, 44.25, 47.65 };
  double[] distance = new double[] { 1.790, 1.900, 2.050, 2.200, 2.558, 2.671, 2.708, 2.969, 3.000, 3.178, 3.300, 3.573, 3.622, 3.830, 4.214, 4.446, 4.645, 4.947};
  double[] velocity = new double[] { 40.00, 41.50, 42.00, 43.00, 44.50, 46.50, 46.75, 47.75, 48.30, 48.50, 48.50, 53.25, 54.40, 55.90, 57.50, 58.75, 59.60, 61.75};

  /** Creates a new Shooter. */

  public Shooter() {
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    Slot0Configs slot0 = config.Slot0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 90;
    config.CurrentLimits.SupplyCurrentLimit = 60;

    // Onboard PID Controller courtesy of 6328
    // slot0.kP = .3;
    // slot0.kD = 0.0;
    // slot0.kS = 0.18;
    // slot0.kV = .1096;
    // config.MotorOutput.PeakForwardDutyCycle = 1.0;
    // config.MotorOutput.PeakReverseDutyCycle = -1.0;
    slot0.kP = 9999;
    config.MotorOutput.PeakForwardDutyCycle = 1.0;
    config.MotorOutput.PeakReverseDutyCycle = 0.0;
    PhoenixUtil.tryUntilOk(5, () -> rightShooter.getConfigurator().apply(config));

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> leftShooter.getConfigurator().apply(config));
    leftShooter.setControl(new Follower(rightShooter.getDeviceID(), MotorAlignmentValue.Opposed));

    tunableRPS = new LoggedTunableNumber("tunableRPS", 0.00);
  }

  @Override
  public void periodic() {
    // Logger.recordOutput(getName() + "/equationVelocity", equationVelocity);
    // Logger.recordOutput(getName() + "/passedThroughDistance",
    // passedThroughDistance);
    // Logger.recordOutput(getName() + "/rightShooterStatorCurrent",
    // rightShooter.getStatorCurrent().getValueAsDouble());
    // Logger.recordOutput(getName() + "/leftShooterStatorCurrent",
    // leftShooter.getStatorCurrent().getValueAsDouble());

  }

  @AutoLogOutput(key = "Shooter/leftVelocity")
  public double getLeftShooterVelocity() {
    return leftShooter.getVelocity().getValueAsDouble();
  }

  @AutoLogOutput(key = "Shooter/rightVelocity")
  public double getRightShooterVelocity() {
    return rightShooter.getVelocity().getValueAsDouble();
  }

  public void setDutyCycleSetpoint(double sp) {
    rightShooter.set(sp);
    if (!rightShooter.isConnected()) leftShooter.set(sp);
  }

  public Command setDutyCycleSetpointCommand(double sp) {
    return run(() -> {
      setDutyCycleSetpoint(sp);
    });
  }

  // public void setBangBangSetpoint(double sp) {
  //   //rightShooter.setControl(bangBangOnboard.withVelocity(sp));
  //   if(rightShooter.getVelocity().getValueAsDouble() < sp){
  //     rightShooter.set(1);
  //   }
  //   else{
  //     rightShooter.set(0.5);
  //   }
  // }

  public void setBangBangSetpoint(double sp) {
    rightShooter.setControl(bangBangOnboard.withVelocity(sp));
    if (!rightShooter.isConnected()) leftShooter.setControl(bangBangOnboard.withVelocity(sp));
  }
  public void setVoltage(double v) {
    rightShooter.setVoltage(v);
    if (!rightShooter.isConnected()) leftShooter.setVoltage(v);
  }

  public void setPIDSetpoint(double sp) {
    rightShooter.setControl(pidVoltage.withVelocity(sp));
    if (!rightShooter.isConnected()) leftShooter.setControl(pidVoltage.withVelocity(sp));
  }

  public Command setBangBangSetpointCommand(double sp) {
    return run(() -> {
      setBangBangSetpoint(sp);
    });
  }

  public Command setVoltageCommand(double v) {
    return run(() -> {
      setVoltage(v);
    });
  }

  public Command setTunableVelocityBangBangCommand() {
    return run(() -> {
      setBangBangSetpoint(tunableRPS.get());
    });
  }
  
  public Command setTunableVelocityPIDCommand() {
    return run(() -> {
      setPIDSetpoint(tunableRPS.get());
    });
  }

  public double veloInterpolate(double d) {
    if (d < distance[0]) return velocity[0];
    else if (d > distance[distance.length - 1]) return velocity[velocity.length - 1];
    else {
      for (int i = 1; i < distance.length; i++) {
        if (distance[i] > d) return (d - distance[i - 1]) / (distance[i] - distance[i - 1]) * (velocity[i] - velocity[i - 1]) + velocity[i - 1];
      }
    }
    return 0;
  }

  public double getVelo(double d) {
    return veloInterpolate(d);
  }
}