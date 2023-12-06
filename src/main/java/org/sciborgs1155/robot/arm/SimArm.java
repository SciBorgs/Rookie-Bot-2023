// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimArm implements JointIO {
  private final SingleJointedArmSim armSim;

  public SimArm() {
    armSim = new SingleJointedArmSim(null, 0, 0, 0, 0, 0, false);
  }

  @Override
  public double getAngle() {
    return 0;
  }

  @Override
  public State getCurrentState() {
    return new State();
  }

  @Override
  public State calculateGoalFromDistance(double distance) {
    return new State();
  }

  @Override
  public void setState(State setpoint) {}

  @Override
  public void close() {}
}
