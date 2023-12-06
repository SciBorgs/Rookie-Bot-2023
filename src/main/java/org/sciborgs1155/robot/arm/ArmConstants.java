// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import org.sciborgs1155.lib.constants.ArmFFConstants;
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.robot.Constants;

/** Add your docs here. */
public class ArmConstants {
  public static final double CONVERSION = 2.0 * Math.PI / Constants.THROUGHBORE_PPR;
  public static final Constraints CONSTRAINTS = new Constraints(3, 2);

  public static final ArmFFConstants FF = new ArmFFConstants(0, 0, 0);
  public static final PIDConstants PID = new PIDConstants(0, 0, 0);
}
