package org.sciborgs1155.robot.intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import org.sciborgs1155.lib.constants.ArmFFConstants;
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.robot.Constants;

public class IntakeConstants {
  public static final double INTAKE_SPEED = 0;
  public static final double OUTTAKE_SPEED = -0;

  public static final double CONVERSION = 2.0 * Math.PI / Constants.THROUGHBORE_PPR;
  public static final Constraints CONSTRAINTS = new Constraints(3, 2);

  public static final ArmFFConstants FF = new ArmFFConstants(0, 0, 0);
  public static final PIDConstants PID = new PIDConstants(0, 0, 0);
}
