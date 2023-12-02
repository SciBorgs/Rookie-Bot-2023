// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.drive;

import static org.sciborgs1155.robot.Ports.Drive.*;
import static org.sciborgs1155.robot.drive.DriveConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.sciborgs1155.lib.constants.SparkUtils;
// create a file and just copy + paste all of that there later

public class Drive extends SubsystemBase {

  //Right and Left side encoders
  



  private final CANSparkMax FRmotor =
      SparkUtils.create(
          FRdrivePort,
          s -> {
            s.setInverted(false);
            s.setIdleMode(IdleMode.kBrake);
            s.setOpenLoopRampRate(0);
            s.setSmartCurrentLimit(50);
          });

  private final CANSparkMax FLmotor =
      SparkUtils.create(
          FLdrivePort,
          s -> {
            s.setInverted(false);
            s.setIdleMode(IdleMode.kBrake);
            s.setOpenLoopRampRate(0);
            s.setSmartCurrentLimit(50);
          });
  private final CANSparkMax MRmotor =
      SparkUtils.create(
          MRdrivePort,
          s -> {
            s.setInverted(false);
            s.setIdleMode(IdleMode.kBrake);
            s.setOpenLoopRampRate(0);
            s.setSmartCurrentLimit(50);
            s.follow(FRmotor);
          });

  private final CANSparkMax MLmotor =
      SparkUtils.create(
          MLdrivePort,
          s -> {
            s.setInverted(false);
            s.setIdleMode(IdleMode.kBrake);
            s.setOpenLoopRampRate(0);
            s.setSmartCurrentLimit(50);
            s.follow(FLmotor);
          });

  private final CANSparkMax BRmotor =
      SparkUtils.create(
          BRdrivePort,
          s -> {
            s.setInverted(false);
            s.setIdleMode(IdleMode.kBrake);
            s.setOpenLoopRampRate(0);
            s.setSmartCurrentLimit(50);
            s.follow(FRmotor);
          });

  private final CANSparkMax BLmotor =
      SparkUtils.create(
          BLdrivePort,
          s -> {
            s.setInverted(false);
            s.setIdleMode(IdleMode.kBrake);
            s.setOpenLoopRampRate(0);
            s.setSmartCurrentLimit(50);
            s.follow(FLmotor);
          });

  /**
   * Encoders PID and FF controllers 
   * DifferentialDriveOdometry  _>> working on this rn
   * Getters for pose, encoder values
   * Setters for setpoints and other important values
   * Style
   */
  //setting up pose origin, facing in pos-X direction
  private final Pose2d pose = new Pose2d();
  private final DifferentialDrivePoseEstimator odometry = 
    new DifferentialDrivePoseEstimator(
      // null, 
      // null, 
      // null, 
      // null, 
      // pose
    )

//Parameters:

// kinematics A correctly-configured kinematics object for your drivetrain.

// gyroAngle The current gyro angle.

// leftDistanceMeters The distance traveled by the left encoder.

// rightDistanceMeters The distance traveled by the right encoder.

// initialPoseMeters The starting pose estimate.

  /** Creates a new Drive. */
  public Drive() {}

  public void setVoltage(Supplier<Double> voltageR, Supplier<Double> voltageL) {
    FRmotor.setVoltage(voltageR.get() * MAX_SPEED);
    FLmotor.setVoltage(voltageL.get() * MAX_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
