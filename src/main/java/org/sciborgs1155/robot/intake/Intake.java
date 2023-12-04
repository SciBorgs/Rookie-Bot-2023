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
    pid = new PIDController(PID.p(), PID.i(), PID.d());
    ff = new ArmFeedforward(FF.s(), FF.g(), FF.v(), FF.a());

    rotationEncoder.setDistancePerPulse(CONVERSION);
  }

  /**
   * Gets the position of the arm.
   *
   * @return The current position of the arm, in radians
   */
  public double getAngle() {
    return rotationEncoder.getDistance();
  }

  /**
   * Gets the state of the arm.
   *
   * @return The current state.
   */
  public State getCurrentState() {
    return new State(getAngle(), rotationEncoder.getRate());
  }

  /**
   * Moves the arm to a desired radian.
   *
   * @param goal
   * @return The command to move the arm to the desired angle.
   */
  public CommandBase goTo(double goal) {
    return followProfile(new State(goal, 0));
  }

  /**
   * Moves the arm to a desired state.
   *
   * @param goal
   * @return The command to move the arm to the desired state.
   */
  public CommandBase goTo(State state) {
    return followProfile(state);
  }

  /**
   * Moves arm directly to a goal given a distance.
   *
   * @param distance
   * @return The command to move to the goal.
   */
  public CommandBase goToAngleFromDistance(double distance) {
    return followProfile(calculateGoalFromDistance(distance));
  }

  /**
   * Moves the arm towards a state.
   *
   * @param setpoint
   * @return The command to move the arm to the desired state.
   */
  private CommandBase setState(State setpoint) {
    double feedforward = ff.calculate(setpoint.position, setpoint.velocity);
    double feedback = pid.calculate(rotationEncoder.getRate(), setpoint.velocity);
    return run(() -> rotateMotor.setVoltage(feedforward + feedback));
  }

  /**
   * Uses a trapezoidal profile to smoothen movement to a goal by successively traveling to
   * intermediate setpoints.
   *
   * @param goal
   * @return The command to run the TrapezoidProfile
   */
  private CommandBase followProfile(State goal) {
    return new TrapezoidProfileCommand(
        new TrapezoidProfile(CONSTRAINTS, goal, getCurrentState()), this::setState);
  }

  /**
   * Uses a best-fit line to calculate the arm state necessary to shoot cubes a certain distance
   * (m/s). TODO
   *
   * @param distance
   * @return The desired goal.
   */
  public State calculateGoalFromDistance(double distance) {
    return new State();
  }

  public CommandBase intake() {
    return run(() -> wheels.set(INTAKE_SPEED));
  }

  public CommandBase outtake() {
    return run(() -> wheels.set(OUTTAKE_SPEED));
  }

  @Override
  public void periodic() {}
}
