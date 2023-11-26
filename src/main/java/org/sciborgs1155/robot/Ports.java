package org.sciborgs1155.robot;

public final class Ports {

  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
    public static final int LEFT_STICK = 2;
    public static final int RIGHT_STICK = 3;
  }

  public static final class Intake {
    public static final int ROTATION_MOTOR = -1;
    public static final int WHEEL_MOTOR = -1;
    public static final int[] ENCODER = new int[] {-1, -1};
  }
}
