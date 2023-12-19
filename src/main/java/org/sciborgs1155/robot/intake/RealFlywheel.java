// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.intake;

import static org.sciborgs1155.robot.Ports.Intake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.MathUtil;
import org.sciborgs1155.lib.constants.SparkUtils;

public class RealFlywheel implements FlywheelIO {
  private final CANSparkMax wheels =
      SparkUtils.create(
          WHEEL_MOTOR,
          s -> {
            s.setIdleMode(IdleMode.kBrake);
            s.setSmartCurrentLimit(40);
            s.setOpenLoopRampRate(0);
          });

  @Override
  public void setSpeed(double speed) {
    wheels.set(MathUtil.clamp(speed, -1, 1));
  }

  @Override
  public void close() {
    wheels.close();
  }
}
