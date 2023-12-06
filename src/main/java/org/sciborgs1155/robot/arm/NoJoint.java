// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class NoJoint implements JointIO {
  public NoJoint() {}

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
