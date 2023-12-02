// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.drive;

import static org.sciborgs1155.robot.drive.DriveConstants.*;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.List;
import java.util.function.Supplier;

import javax.management.ConstructorParameters;

import org.sciborgs1155.lib.failure.Fallible;
import org.sciborgs1155.lib.failure.HardwareFault;
import org.sciborgs1155.robot.Ports.DrivePorts;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

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

  // Gyro
  @Log private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(DrivePorts.GYRO_PORT);
  //Right and Left side encoders
  @Log private final RelativeEncoder rightEncoder = FRmotor.getEncoder();
  @Log private final RelativeEncoder leftEncoder = FLmotor.getEncoder();

  @Log private double heading;
  
  @Log private final PIDController RdrivePID = new PIDController(DrivePorts.kP, DrivePorts.kI, DrivePorts.kD);
  @Log private final PIDController LdrivePID = new PIDController(DrivePorts.kP, DrivePorts.kI, DrivePorts.kD);

  @Log private final SimpleMotorFeedforward RdriveFF = new SimpleMotorFeedforward(DrivePorts.kS, DrivePorts.kV, DrivePorts.kA);
  @Log private final SimpleMotorFeedforward LdriveFF = new SimpleMotorFeedforward(DrivePorts.kS, DrivePorts.kV, DrivePorts.kA);

  @Log private double setPoint;
  

  /**
   * Encoders PID and FF controllers 
   * DifferentialDriveOdometry  _>> working on this rn
   * Getters for pose, encoder values
   * Setters for setpoints and other important values
   * Style
   */
  

  //setting up pose origin, facing in pos-X direction
  private final Pose2d pose = new Pose2d();
  //sets up Odometry
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(ROBOT_TRACK);

  private final DifferentialDrivePoseEstimator odometry = 
    new DifferentialDrivePoseEstimator(
      kinematics,
      gyro.getRotation2d(), 
      leftEncoder.getPosition(), 
      rightEncoder.getPosition(), 
      pose);
    

//Parameters:
// kinematics A correctly-configured kinematics object for your drivetrain.
// gyroAngle The current gyro angle.

  /** Creates a new Drive. */
  public Drive(){
    leftEncoder.setPositionConversionFactor(DriveConstants.GEAR_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE);
    rightEncoder.setPositionConversionFactor(DriveConstants.GEAR_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE);

  }

  public void setVoltage(Supplier<Double> voltageR, Supplier<Double> voltageL) {
    FRmotor.setVoltage(voltageR.get() * MAX_SPEED);
    FLmotor.setVoltage(voltageL.get() * MAX_SPEED);
  }

  public void setSpeed(DifferentialDriveWheelSpeeds speeds){
    double lFF = LdriveFF.calculate(speeds.leftMetersPerSecond);
    double rFF = RdriveFF.calculate(speeds.rightMetersPerSecond);

    double lFB = LdrivePID.calculate(leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
    double rFB = RdrivePID.calculate(rightEncoder.getVelocity(), speeds.rightMetersPerSecond);

    setVoltage(() -> rFF+rFB, () -> lFF+lFB);
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }
  // either "right" "Right" "left" or "Left" is available for String side, otherwise returns null

  public RelativeEncoder getEncoder(String side){
    return side.equals("right") ||
     side.equals("Right") 
     ? rightEncoder 
     : (side.equals("left") || 
      side.equals("Left") 
      ? leftEncoder 
      : null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    //wheel speeds
    DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());

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
