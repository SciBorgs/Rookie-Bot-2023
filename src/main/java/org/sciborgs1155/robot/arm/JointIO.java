package org.sciborgs1155.robot.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public interface JointIO {

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
  public State calculateGoalFromDistance(double distance);

  public void close();
}
