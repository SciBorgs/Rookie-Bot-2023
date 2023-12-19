package org.sciborgs1155.robot.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import org.sciborgs1155.lib.constants.ArmFFConstants;
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.robot.Constants;

public class ArmConstants {
  public static final double CONVERSION = 2.0 * Math.PI / Constants.THROUGHBORE_PPR;
  public static final Constraints CONSTRAINTS = new Constraints(3, 2);

  public static final ArmFFConstants FF = new ArmFFConstants(0, 0, 0);
  public static final PIDConstants PID = new PIDConstants(0, 0, 0);

  public static final DCMotor GEARBOX = DCMotor.getNEO(-1);
  public static final double GEARING = -1;

  public static final double ARM_LENGTH = -1;
  public static final double ARM_MASS = -1;

  public static final double MIN_ANGLE = -1;
  public static final double MAX_ANGLE = -1;
}
