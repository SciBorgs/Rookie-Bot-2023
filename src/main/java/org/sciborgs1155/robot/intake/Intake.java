// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.intake;

import static org.sciborgs1155.robot.Ports.Intake.ENCODER;
import static org.sciborgs1155.robot.Ports.Intake.WHEEL_MOTOR;
import static org.sciborgs1155.robot.intake.IntakeConstants.CONVERSION;
import static org.sciborgs1155.robot.intake.IntakeConstants.INTAKE_SPEED;
import static org.sciborgs1155.robot.intake.IntakeConstants.OUTTAKE_SPEED;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.lib.constants.SparkUtils;

public class Intake extends SubsystemBase {
  
  public Intake() {}

  public CommandBase intake() {
    return run(() -> wheels.set(INTAKE_SPEED));
  }

  public CommandBase outtake() {
    return run(() -> wheels.set(OUTTAKE_SPEED));
  }

  @Override
  public void periodic() {}
}
