package org.sciborgs1155.robot.intake;

import static org.sciborgs1155.robot.intake.FlywheelConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase implements AutoCloseable {
  private final FlywheelIO intake;

  public Flywheel(FlywheelIO intake) {
    this.intake = intake;
  }

  public static Flywheel create() {
    return new Flywheel(new RealFlywheel());
  }

  public static Flywheel createNone() {
    return new Flywheel(new NoFlywheel());
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

  @Override
  public void close() throws Exception {
    intake.close();
  }
}
