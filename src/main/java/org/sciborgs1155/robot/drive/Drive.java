// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.drive;

import static org.sciborgs1155.robot.Ports.DrivePorts;
import static org.sciborgs1155.robot.drive.DriveConstants.*;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.nio.channels.Channel;
import java.util.function.Supplier;
import org.sciborgs1155.lib.constants.SparkUtils;

import org.sciborgs1155.lib.failure.Fallible;
import org.sciborgs1155.robot.Ports.DrivePorts;

public class Drive extends SubsystemBase implements Fallible, Loggable, AutoCloseable{
  // Gyro
  @Log private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(0);
  //Right and Left side encoders
  @Log private final AnalogEncoder rightEncoder = new AnalogEncoder(DrivePorts.rightEncoderPort);
  @Log private final AnalogEncoder leftEncoder = new AnalogEncoder(DrivePorts.leftEncoderPort);


//creates StandardDriveMotor
  StandardDriveMotor driveMotor = new StandardDriveMotor();
//new formatting, check out StandardDriveMotor for more; just added so it is less messy in this document
  private final CANSparkMax FRmotor = driveMotor.create(DrivePorts.FRdrivePort);
  private final CANSparkMax FLmotor = driveMotor.create(DrivePorts.FLdrivePort);
  private final CANSparkMax MRmotor = driveMotor.create(DrivePorts.MRdrivePort);
  private final CANSparkMax MLmotor = driveMotor.create(DrivePorts.MLdrivePort);
  private final CANSparkMax BRmotor = driveMotor.create(DrivePorts.BRdrivePort);
  private final CANSparkMax BLmotor = driveMotor.create(DrivePorts.BLdrivePort);

  /**
   * Encoders PID and FF controllers 
   * DifferentialDriveOdometry  _>> working on this rn
   * Getters for pose, encoder values
   * Setters for setpoints and other important values
   * Style
   */
  
  //setting up pose origin, facing in pos-X direction
  private final Pose2d pose = new Pose2d();
  //sets up Pose Estimator
  private final DifferentialDrivePoseEstimator odometry = 
    new DifferentialDrivePoseEstimator(
      null, 
      null, 
      leftEncoder.getDistance(), 
      rightEncoder.getDistance(), 
      pose
    )

//Parameters:
// kinematics A correctly-configured kinematics object for your drivetrain.
// gyroAngle The current gyro angle.

  /** Creates a new Drive. */
  public Drive() {}

  public void setVoltage(Supplier<Double> voltageR, Supplier<Double> voltageL) {
    FRmotor.setVoltage(voltageR.get() * MAX_SPEED);
    FLmotor.setVoltage(voltageL.get() * MAX_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(null, leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  @Override
  .close();
}
