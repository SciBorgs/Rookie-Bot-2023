// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.arm;

import static org.sciborgs1155.robot.arm.ArmConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.sciborgs1155.robot.Constants;

public class SimJoint implements JointIO {
  private final SingleJointedArmSim jointSim;
  private final PIDController pid;
  private final ArmFeedforward ff;

  public SimJoint() {
    jointSim =
        new SingleJointedArmSim(
            GEARBOX,
            GEARING,
            SingleJointedArmSim.estimateMOI(ARM_LENGTH, ARM_MASS),
            ARM_LENGTH,
            MIN_ANGLE,
            MAX_ANGLE,
            true);
    pid = new PIDController(PID.p(), PID.i(), PID.d());
    ff = new ArmFeedforward(FF.s(), FF.g(), FF.v(), FF.a());
  }

  @Override
  public double getAngle() {
    return jointSim.getAngleRads();
  }

  @Override
  public double getVelocity() {
    return jointSim.getVelocityRadPerSec();
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
    jointSim.setInputVoltage(feedforward + feedback);
    jointSim.update(Constants.PERIOD);
  }

  @Override
  public void close() {}
}
