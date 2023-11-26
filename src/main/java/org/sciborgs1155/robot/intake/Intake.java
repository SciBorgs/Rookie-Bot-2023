// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.intake;

import static org.sciborgs1155.robot.Ports.Intake.*;
import static org.sciborgs1155.robot.intake.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import org.sciborgs1155.lib.constants.SparkUtils;

public class Intake extends SubsystemBase {

  private final CANSparkMax rotateMotor =
      SparkUtils.create(
          ROTATION_MOTOR,
          s -> {
            s.setIdleMode(IdleMode.kBrake);
            s.setSmartCurrentLimit(40);
            s.setOpenLoopRampRate(0);
          });

  private final CANSparkMax wheels =
      SparkUtils.create(
          WHEEL_MOTOR,
          s -> {
            s.setIdleMode(IdleMode.kBrake);
            s.setSmartCurrentLimit(40);
            s.setOpenLoopRampRate(0);
          });

  private final PIDController pid;
  private final ArmFeedforward ff;
  private final Encoder rotationEncoder;

  public Intake() {
    rotationEncoder = new Encoder(ENCODER[0], ENCODER[1]);
    pid = new PIDController(PID.kP, PID.kI, PID.kD);
    ff = new ArmFeedforward(FF.s(), FF.g(), FF.v(), FF.a());

    rotationEncoder.setDistancePerPulse(CONVERSION);
  }

  public double getAngle() {
    return rotationEncoder.getDistance();
  }

  public State getCurrentState() {
    return new State(getAngle(), rotationEncoder.getRate());
  }

  public State getDesiredState(double distance) {
    return new State(); // func(distance) = desiredAngle --> convert to desired state
  }

  public CommandBase followProfile(State goal) {
    return new TrapezoidProfileCommand(
        new TrapezoidProfile(CONSTRAINTS, goal, getCurrentState()), this::setAngle);
  }

  public Command setAngle(State setpoint) {
    double feedforward = ff.calculate(setpoint.position, setpoint.velocity);
    double feedback = pid.calculate(getAngle(), setpoint.position);
    return run(() -> rotateMotor.setVoltage(feedforward + feedback));
  }

  public Command intake() {
    return run(() -> wheels.set(INTAKE_SPEED));
  }

  public Command outtake() {
    return run(() -> wheels.set(OUTTAKE_SPEED));
  }

  @Override
  public void periodic() {}
}
