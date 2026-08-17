package frc.robot.commands;

import static frc.robot.Constants.IntakeConstants.INTAKE_AGITATE_DOWN;

import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.transfer.Transfer;

public class AgitateCommand extends Command {

  private final Intake intake;
  private final Transfer transfer;
  private final Supplier<Trigger> speedBoostSupplier;
  private final BooleanSupplier sotmSupplier;
  private final DoubleSupplier feedPowerSupplier;
  boolean auto = false;

  private static final double SPEED_BOOST_FACTOR = 2.5;
  private static final double AGITATE_RATE = 6.0;
  private static final double AGITATE_MIN_POS = 22;
  private static final double TIMER_DELAY = 0.5;
  double lastAgigatePos = INTAKE_AGITATE_DOWN;

  private double agitatePos = INTAKE_AGITATE_DOWN;
  private double multiplier = 3.5;
  private double offset = 0;

  public AgitateCommand(Intake intake, Transfer transfer, Supplier<Trigger> speedBoostSupplier,
      BooleanSupplier sotmSupplier, DoubleSupplier multiplier, DoubleSupplier feedPowerSupplier) {
    this.intake = intake;
    this.transfer = transfer;
    this.speedBoostSupplier = speedBoostSupplier;
    this.sotmSupplier = sotmSupplier;
    this.feedPowerSupplier = feedPowerSupplier;
    this.multiplier = multiplier.getAsDouble();
    addRequirements(intake);
  }

  public AgitateCommand(Intake intake, Transfer transfer, Supplier<Trigger> speedBoostSupplier,
      BooleanSupplier sotmSupplier, DoubleSupplier multiplier) {
    this(intake, transfer, speedBoostSupplier, sotmSupplier, multiplier, () -> 1.0);
  }
  public AgitateCommand(Intake intake, Transfer transfer, BooleanSupplier speedBoostSupplier,
      BooleanSupplier sotmSupplier, DoubleSupplier multiplier, boolean auto) {
    this(intake, transfer, null, sotmSupplier, multiplier, () -> 1.0);
    this.auto = auto;
  }

  @Override
  public void initialize() {
    agitatePos = INTAKE_AGITATE_DOWN;
    offset = 0;
    intake.setIntakePosition(INTAKE_AGITATE_DOWN);
    intake.setIntakeDutyCycle(0);
 }

  @Override
  public void execute() {
    
    boolean speedBoost = false;
    if(speedBoostSupplier != null && !auto && speedBoostSupplier.get() != null) {
      speedBoost = speedBoostSupplier.get().getAsBoolean();
    }
    double feedTime = transfer.feedTimer.get();


    if (feedTime >= TIMER_DELAY || speedBoost) {
      double activeMultiplier = (feedTime >= 2.5 || speedBoost) ? SPEED_BOOST_FACTOR : 1.0;
      double elapsed = feedTime - TIMER_DELAY;

      if (offset != 0 && !speedBoost) {
        agitatePos = Math.max(AGITATE_MIN_POS, INTAKE_AGITATE_DOWN + offset - (elapsed * AGITATE_RATE));
      } else {
        agitatePos = Math.max(AGITATE_MIN_POS, INTAKE_AGITATE_DOWN - (elapsed * AGITATE_RATE * activeMultiplier));
      }

      if (speedBoost) {
        offset = intake.getIntakePosition() - Math.max(AGITATE_MIN_POS, INTAKE_AGITATE_DOWN - (elapsed * AGITATE_RATE));
      }

      if (sotmSupplier.getAsBoolean()) {
        intake.setIntakeDutyCycle(0.5);
      }
    }

    double scaledAgitatePos = INTAKE_AGITATE_DOWN - (INTAKE_AGITATE_DOWN - agitatePos) * feedPowerSupplier.getAsDouble();
    lastAgigatePos = Math.min(lastAgigatePos, scaledAgitatePos);
    intake.setIntakePosition(scaledAgitatePos);
    intake.setIntakeDutyCycle(0.5);

    Logger.recordOutput(getName() + "/feedTimer", feedTime);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
