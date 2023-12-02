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
    public static final int FRdrivePort = -1;
    public static final int FLdrivePort = -1;
    public static final int MRdrivePort = -1;
    public static final int MLdrivePort = -1;
    public static final int BRdrivePort = -1;
    public static final int BLdrivePort = -1;

    public static final int rightEncoderPort = -1;
    public static final int leftEncoderPort = -1;
  }
}
