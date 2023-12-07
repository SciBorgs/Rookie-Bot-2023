// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.intake;

import static org.sciborgs1155.robot.Ports.Intake.*;
import static org.sciborgs1155.robot.intake.IntakeConstants.*;

import org.sciborgs1155.lib.constants.SparkUtils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Encoder;

public class RealIntake implements IntakeIO {
    private final CANSparkMax wheels =
      SparkUtils.create(
          WHEEL_MOTOR,
          s -> {
            s.setIdleMode(IdleMode.kBrake);
            s.setSmartCurrentLimit(40);
            s.setOpenLoopRampRate(0);
          });

  private final Encoder rotationEncoder;

  public RealIntake() {
    rotationEncoder = new Encoder(ENCODER[0], ENCODER[1]);

    rotationEncoder.setDistancePerPulse(CONVERSION);
  }
}
