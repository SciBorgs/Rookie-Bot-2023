package org.sciborgs1155.robot.drive;

import static org.sciborgs1155.robot.drive.DriveConstants.*;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.jni.CANSparkMaxJNI;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.List;
import java.util.function.Supplier;

import org.sciborgs1155.lib.failure.Fallible;
import org.sciborgs1155.lib.failure.HardwareFault;
import org.sciborgs1155.robot.Ports.DrivePorts;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Drive extends SubsystemBase implements Fallible, Loggable, AutoCloseable{

  //creates StandardDriveMotor
  StandardDriveMotor driveMotor = new StandardDriveMotor();
//new formatting, check out StandardDriveMotor for more; just added so it is less messy in this document
  private final CANSparkMax FRmotor = driveMotor.create(DrivePorts.FR_DRIVE_PORT);
  private final CANSparkMax FLmotor = driveMotor.create(DrivePorts.FL_DRIVE_PORT);
  private final CANSparkMax MRmotor = driveMotor.create(DrivePorts.MR_DRIVE_PORT);
  private final CANSparkMax MLmotor = driveMotor.create(DrivePorts.ML_DRIVE_PORT);
  private final CANSparkMax BRmotor = driveMotor.create(DrivePorts.BL_DRIVE_PORT);
  private final CANSparkMax BLmotor = driveMotor.create(DrivePorts.BL_DRIVE_PORT);

  private final CANSparkMax[] rightSparks = {FRmotor, MRmotor, BRmotor};
  private final CANSparkMax[] leftSparks = {FLmotor, MLmotor, BLmotor};

  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightSparks);
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftSparks);

  // Gyros
  @Log private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(DrivePorts.GYRO_PORT);
  //Right and Left side encoders
  @Log private final RelativeEncoder rightEncoder = FRmotor.getEncoder();
  @Log private final RelativeEncoder leftEncoder = FLmotor.getEncoder();

  @Log private double heading;
  
  @Log private final PIDController RdrivePID = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
  @Log private final PIDController LdrivePID = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

  @Log private final SimpleMotorFeedforward RdriveFF = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);
  @Log private final SimpleMotorFeedforward LdriveFF = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

  

  /**
   * Encoders PID and FF controllers, -> mostly done by now, maybe there is something else about this that can be worked on later
   * DifferentialDriveOdometry -> used DifferentialDrivePoseEstimator instead rn
   * Getters for pose, encoder values
   * Setters for setpoints and other important values
   * Style
   */
  

  //sets up Odometry
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(ROBOT_TRACK);
  DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(0, 0);

  @Log private final DifferentialDrivePoseEstimator odometry = 
    new DifferentialDrivePoseEstimator(
      kinematics,
      gyro.getRotation2d(), 
      leftEncoder.getPosition(), 
      rightEncoder.getPosition(), 
      new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), //these could probably be adjusted later, taken from wpilib docs
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  /** Creates a new Drive. */
  public Drive(){
    leftEncoder.setPositionConversionFactor(DriveConstants.GEAR_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE);
    rightEncoder.setPositionConversionFactor(DriveConstants.GEAR_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE);

    leftEncoder.setVelocityConversionFactor(DriveConstants.GEAR_RATIO);
    leftEncoder.setVelocityConversionFactor(DriveConstants.GEAR_RATIO);

    rightMotors.setInverted(true);
  }

  public void setVoltage(Supplier<Double> voltageR, Supplier<Double> voltageL) {
    rightMotors.setVoltage(voltageR.get() * MAX_SPEED);
    leftMotors.setVoltage(voltageL.get() * MAX_SPEED);
  }

  public void setSpeed(DifferentialDriveWheelSpeeds speeds){
    double lFF = LdriveFF.calculate(speeds.leftMetersPerSecond);
    double rFF = RdriveFF.calculate(speeds.rightMetersPerSecond);

    double lFB = LdrivePID.calculate(leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
    double rFB = RdrivePID.calculate(rightEncoder.getVelocity(), speeds.rightMetersPerSecond);

    setVoltage(() -> rFF+rFB, () -> lFF+lFB);
  }

  public double[] getVelocity(){
    double[] velocities = {rightEncoder.getVelocity(), leftEncoder.getVelocity()};
    return velocities;
  }

  public Rotation2d getRotation() {
    return gyro.getRotation2d();
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public double getPoseDegrees() {
    return getPose().getRotation().getDegrees();
  }
  
  // either "right" "Right" "left" or "Left" is available for String side, otherwise returns null

  public RelativeEncoder getEncoder(String side){
    return side.toLowerCase().equals("right")
     ? rightEncoder 
     : (side.toLowerCase().equals("left")
      ? leftEncoder 
      : null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    //wheel speeds
    speeds = new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());

  }

  @Override
  //closing everything that needs to be closed
  public void close() throws Exception {
    FRmotor.close();
    FLmotor.close();
    MRmotor.close();
    MLmotor.close();
    BRmotor.close();
    BLmotor.close();
    gyro.close();
  }

  //add getFaults() here later

  public List<HardwareFault> getFaults(){
    
  }
}
