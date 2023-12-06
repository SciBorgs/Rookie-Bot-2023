package org.sciborgs1155.robot.drive;

import static org.sciborgs1155.robot.drive.DriveConstants.*;
import static org.sciborgs1155.robot.Ports.DrivePorts.*;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.jni.CANSparkMaxJNI;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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


public class Drive extends SubsystemBase implements Loggable, AutoCloseable{
  //creates StandardDriveMotor
  StandardDriveMotor driveMotor = new StandardDriveMotor();
//new formatting, check out StandardDriveMotor for more; just added so it is less messy in this document
  private final CANSparkMax fRightMotor = driveMotor.create(FR_DRIVE_PORT);
  private final CANSparkMax fLeftMotor = driveMotor.create(FL_DRIVE_PORT);
  private final CANSparkMax bRightMotor = driveMotor.create(BL_DRIVE_PORT);
  private final CANSparkMax bLeftMotor = driveMotor.create(BL_DRIVE_PORT);

  private final CANSparkMax[] rightSparks = {fRightMotor, bRightMotor};
  private final CANSparkMax[] leftSparks = {fLeftMotor, bLeftMotor};

  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightSparks);
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftSparks);

  // Gyros
  @Log private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(DrivePorts.PIGEON);
  //Right and Left side encoders
  @Log private final RelativeEncoder rightEncoder = fRightMotor.getEncoder();
  @Log private final RelativeEncoder leftEncoder = fLeftMotor.getEncoder();

  @Log private double heading;
  
  @Log private final PIDController RdrivePID = new PIDController(kP, kI, kD);
  @Log private final PIDController LdrivePID = new PIDController(kP,  kI, kD);

  @Log private final SimpleMotorFeedforward RdriveFF = new SimpleMotorFeedforward(kS, kV, kA);
  @Log private final SimpleMotorFeedforward LdriveFF = new SimpleMotorFeedforward(kS, kV, kA);

  

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
    leftEncoder.setPositionConversionFactor(GEAR_RATIO * WHEEL_CIRCUMFERENCE);
    rightEncoder.setPositionConversionFactor(GEAR_RATIO * WHEEL_CIRCUMFERENCE);

    leftEncoder.setVelocityConversionFactor(GEAR_RATIO);
    leftEncoder.setVelocityConversionFactor(GEAR_RATIO);

    rightMotors.setInverted(true);
    
    // AutoBuilder.configureLTV()

    
  }

  public void setVoltage(double voltageR, double voltageL) {
    rightMotors.setVoltage(MathUtil.clamp(voltageR, -MAX_VOLTAGE, MAX_VOLTAGE)); 
    leftMotors.setVoltage(MathUtil.clamp(voltageL, -MAX_VOLTAGE, MAX_VOLTAGE));
  }

  public void setSpeed(DifferentialDriveWheelSpeeds speeds){
    double lFF = LdriveFF.calculate(speeds.leftMetersPerSecond);
    double rFF = RdriveFF.calculate(speeds.rightMetersPerSecond);

    double lFB = LdrivePID.calculate(leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
    double rFB = RdrivePID.calculate(rightEncoder.getVelocity(), speeds.rightMetersPerSecond);

    setVoltage(rFF+rFB, lFF+lFB);

  }

  public void resetGyro (){
    gyro.reset();
  }

  public Rotation2d getRotation() {
    return gyro.getRotation2d();
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  @Override
  //closing everything that needs to be closed
  public void close() throws Exception {
    fRightMotor.close();
    fLeftMotor.close();
    bRightMotor.close();
    bLeftMotor.close();
    gyro.close();
  }
}
