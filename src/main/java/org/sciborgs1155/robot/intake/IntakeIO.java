package org.sciborgs1155.robot.intake;

public interface IntakeIO extends AutoCloseable {
  /**
   * Run the wheels at a desired speed.
   *
   * @param speed -1 to 1
   */
  public void setSpeed(double speed);
}
