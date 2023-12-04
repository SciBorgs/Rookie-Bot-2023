package org.sciborgs1155.robot;

public final class Ports {

  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
    public static final int LEFT_STICK = 2;
    public static final int RIGHT_STICK = 3;

    public static final int LEFT_JOYSTICK = -1;
    public static final int RIGHT_JOYSTICK = -1;
  }
  
  public static final class DrivePorts {
    // needs to be changed sometime soon
    public static final int FR_DRIVE_PORT = -1;
    public static final int FL_DRIVE_PORT = -1;
    public static final int MR_DRIVE_PORT = -1;
    public static final int ML_DRIVE_PORT = -1;
    public static final int BR_DRIVE_PORT = -1;
    public static final int BL_DRIVE_PORT = -1;

    public static final int RIGHT_ENCODER_PORT = -1;
    public static final int LEFT_ENCODER_PORT = -1;

    public static final int GYRO_PORT = -1;
    
  public static final class Intake {
    public static final int ROTATION_MOTOR = -1;
    public static final int WHEEL_MOTOR = -1;
    public static final int[] ENCODER = new int[] {-1, -1};
  }
}
