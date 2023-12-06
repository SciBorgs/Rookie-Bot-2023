// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public interface JointIO {
  public double getAngle();

  public State getCurrentState();

  public State calculateGoalFromDistance(double distance);

  public void setState(State setpoint);

  public void close();
}
