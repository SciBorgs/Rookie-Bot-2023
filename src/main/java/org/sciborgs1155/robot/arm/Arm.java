package org.sciborgs1155.robot.arm;

import static org.sciborgs1155.robot.arm.ArmConstants.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import org.sciborgs1155.lib.DeferredCommand;
import org.sciborgs1155.robot.Robot;

public class Arm extends SubsystemBase {
  private final JointIO arm;

  public Arm(JointIO arm) {
    this.arm = arm;
  }

  public static Arm create() {
    return Robot.isReal() ? new Arm(new RealJoint()) : new Arm(new SimJoint());
  }

  public static Arm createNone() {
    return new Arm(new NoJoint());
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
    return followProfile(arm.calculateGoalFromDistance(distance));
  }

  /**
   * Uses a trapezoidal profile to smoothen movement to a goal by successively traveling to
   * intermediate setpoints.
   *
   * @param goal
   * @return The command to run the TrapezoidProfile
   */
  private CommandBase followProfile(State goal) {
    return new DeferredCommand(
        () ->
            new TrapezoidProfileCommand(
                new TrapezoidProfile(CONSTRAINTS, goal, arm.getCurrentState()),
                arm::setState,
                this),
        this);
  }
}
