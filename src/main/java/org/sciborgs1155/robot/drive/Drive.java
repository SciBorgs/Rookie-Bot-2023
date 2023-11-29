// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.drive;

import static org.sciborgs1155.robot.Ports.Drive.*;
import static org.sciborgs1155.robot.drive.DriveConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.sciborgs1155.lib.constants.SparkUtils;
// create a file and just copy + paste all of that there later

public class Drive extends SubsystemBase {
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
   * DifferentialDriveOdometry 
   * Getters for pose, encoder values
   * Setters for setpoints and other important values
   * Style
   */

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
