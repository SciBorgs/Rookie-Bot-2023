// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.arm;

import static org.sciborgs1155.robot.arm.ArmConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.sciborgs1155.robot.Constants;

public class SimArm implements JointIO {
  private final SingleJointedArmSim armSim;
  private final PIDController pid;
  private final ArmFeedforward ff;

  public SimArm() {
    armSim =
        new SingleJointedArmSim(
            GEARBOX,
            GEARING,
            SingleJointedArmSim.estimateMOI(ARM_LENGTH, ARM_MASS),
            ARM_LENGTH,
            MIN_ANGLE,
            MAX_ANGLE,
            false);
    pid = new PIDController(PID.p(), PID.i(), PID.d());
    ff = new ArmFeedforward(FF.s(), FF.g(), FF.v(), FF.a());
  }

  @Override
  public double getAngle() {
    return armSim.getAngleRads();
  }

  @Override
  public double getVelocity() {
    return armSim.getVelocityRadPerSec();
  }

  @Override
  public State getCurrentState() {
    return new State(getAngle(), getVelocity());
  }

  @Override
  public State calculateGoalFromDistance(double distance) {
    return new State();
  }

  @Override
  public void setState(State setpoint) {
    double feedforward = ff.calculate(setpoint.position, setpoint.velocity);
    double feedback = pid.calculate(getVelocity(), setpoint.velocity);
    armSim.setInputVoltage(feedforward + feedback);
    armSim.update(Constants.PERIOD);
  }

  @Override
  public void close() {}
}
