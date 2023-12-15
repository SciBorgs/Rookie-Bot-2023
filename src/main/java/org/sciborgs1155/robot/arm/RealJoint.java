package org.sciborgs1155.robot.arm;

import static org.sciborgs1155.robot.Ports.Arm.*;
import static org.sciborgs1155.robot.arm.ArmConstants.CONVERSION;
import static org.sciborgs1155.robot.arm.ArmConstants.FF;
import static org.sciborgs1155.robot.arm.ArmConstants.PID;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Encoder;
import org.sciborgs1155.lib.constants.SparkUtils;

public class RealJoint implements JointIO {
  private final CANSparkMax motor =
      SparkUtils.create(
          ROTATION_MOTOR,
          s -> {
            s.setIdleMode(IdleMode.kBrake);
            s.setSmartCurrentLimit(40);
            s.setOpenLoopRampRate(0);
          });
  // TODO: Align with correct encoder type on real robot
  private final Encoder rotationEncoder = new Encoder(ENCODER[0], ENCODER[1]);
  private final PIDController pid = new PIDController(PID.p(), PID.i(), PID.d());
  private final ArmFeedforward ff = new ArmFeedforward(FF.s(), FF.g(), FF.v(), FF.a());

  public RealJoint() {
    rotationEncoder.setDistancePerPulse(CONVERSION);
  }

  @Override
  public double getAngle() {
    return rotationEncoder.getDistance();
  }

  @Override
  public double getVelocity() {
    return rotationEncoder.getRate();
  }

  @Override
  public State getCurrentState() {
    return new State(getAngle(), getVelocity());
  }

  @Override
  public void setState(State setpoint) {
    double feedforward = ff.calculate(setpoint.position, setpoint.velocity);
    double feedback = pid.calculate(rotationEncoder.getRate(), setpoint.velocity);
    motor.setVoltage(feedforward + feedback);
  }

  @Override
  public void close() {
    motor.close();
  }
}
