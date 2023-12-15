package org.sciborgs1155.robot.intake;

import static org.sciborgs1155.robot.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final IntakeIO intake;

  public Intake(IntakeIO intake) {
    this.intake = intake;
  }

  public static Intake create() {
    return new Intake(new RealIntake());
  }

  public static Intake createNone() {
    return new Intake(new NoIntake());
  }

  public CommandBase setSpeed(double speed) {
    return run(() -> intake.setSpeed(speed));
  }

  public CommandBase intake() {
    return setSpeed(INTAKE_SPEED);
  }

  public CommandBase outtake() {
    return setSpeed(OUTTAKE_SPEED);
  }

  @Override
  public void periodic() {}
}
