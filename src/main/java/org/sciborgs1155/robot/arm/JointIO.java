package org.sciborgs1155.robot.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public interface JointIO extends AutoCloseable, Sendable {

  /**
   * Gets the position of the arm.
   *
   * @return The current position of the arm, in radians
   */
  public double getAngle();

  /**
   * Gets the velocity of the arm.
   *
   * @return The current velocity of the arm, in m/s
   */
  public double getVelocity();

  /**
   * Gets the state of the arm.
   *
   * @return The current state.
   */
  public State getCurrentState();

  /**
   * Moves the arm towards a state.
   *
   * @param setpoint
   * @return The command to move the arm to the desired state.
   */
  public void setState(State setpoint);

  /**
   * Uses a best-fit line to calculate the arm state necessary to shoot cubes a certain distance
   * (m). TODO
   *
   * @param distance
   * @return The desired goal.
   */
  default State calculateGoalFromDistance(double distance) {
    return new State();
  }

  @Override
  default void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("current angle", () -> getAngle(), null);
    builder.addDoubleProperty("current state", () -> getCurrentState().position, null);
    builder.addDoubleProperty("current velocity", () -> getVelocity(), null);
  }
}
