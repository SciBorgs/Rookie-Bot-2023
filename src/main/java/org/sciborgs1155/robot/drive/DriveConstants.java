package org.sciborgs1155.robot.drive;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;
import org.sciborgs1155.lib.constants.BasicFFConstants;
import org.sciborgs1155.lib.constants.PIDConstants;

public final class DriveConstants {

  public static final PathConstraints CONSTRAINTS =
      // new PathConstraints(MAX_SPEED / 1.9, MAX_ACCEL / 1.4);

  public static final class driveMotor {
    public static final class Driving {
      // this will all need to be revised later (put back spacing, substitute values)
            // public static final double GEARING = 1.0 / 45.0 / 22.0 * 15.0 * 14.0;

            // public static final double CONVERSION = CIRCUMFERENCE * GEARING;

            // public static final PIDConstants PID = new PIDConstants(0.11, 0, 0.06);
            // public static final BasicFFConstants FF = new BasicFFConstants(0.3, 2.7, 0.25);
    }
  }
}