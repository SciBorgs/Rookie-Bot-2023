// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.drive;

import static org.sciborgs1155.robot.Ports.DrivePorts;
import static org.sciborgs1155.robot.drive.DriveConstants.*;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.nio.channels.Channel;
import java.util.List;
import java.util.function.Supplier;
import org.sciborgs1155.lib.constants.SparkUtils;

import org.sciborgs1155.lib.failure.Fallible;
import org.sciborgs1155.lib.failure.HardwareFault;
import org.sciborgs1155.robot.Ports.DrivePorts;

public class Drive extends SubsystemBase implements Fallible, Loggable, AutoCloseable{
  // Gyro
  @Log private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(DrivePorts.GYRO_PORT);
  //Right and Left side encoders
  @Log private final AnalogEncoder rightEncoder = new AnalogEncoder(DrivePorts.RIGHT_ENCODER_PORT);
  @Log private final AnalogEncoder leftEncoder = new AnalogEncoder(DrivePorts.LEFT_ENCODER_PORT);

  @Log private double heading;
  
  @Log private PIDController drivePID = new PIDController(DrivePorts.kP, DrivePorts.kI, DrivePorts.kD);
  @Log private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(DrivePorts.kS, DrivePorts.kV, DrivePorts.kA);

  @Log private double setPoint;


//creates StandardDriveMotor
  StandardDriveMotor driveMotor = new StandardDriveMotor();
//new formatting, check out StandardDriveMotor for more; just added so it is less messy in this document
  private final CANSparkMax FRmotor = driveMotor.create(DrivePorts.FR_DRIVE_PORT);
  private final CANSparkMax FLmotor = driveMotor.create(DrivePorts.FL_DRIVE_PORT);
  private final CANSparkMax MRmotor = driveMotor.create(DrivePorts.MR_DRIVE_PORT);
  private final CANSparkMax MLmotor = driveMotor.create(DrivePorts.ML_DRIVE_PORT);
  private final CANSparkMax BRmotor = driveMotor.create(DrivePorts.BL_DRIVE_PORT);
  private final CANSparkMax BLmotor = driveMotor.create(DrivePorts.BL_DRIVE_PORT);

  /**
   * Encoders PID and FF controllers 
   * DifferentialDriveOdometry  _>> working on this rn
   * Getters for pose, encoder values
   * Setters for setpoints and other important values
   * Style
   */
  
  //setting up pose origin, facing in pos-X direction
  private final Pose2d pose = new Pose2d();
  //sets up Pose Estimator
  private final DifferentialDrivePoseEstimator odometry = 
    new DifferentialDrivePoseEstimator(
      null, 
      gyro.getRotation2d(), 
      leftEncoder.getDistance(), 
      rightEncoder.getDistance(), 
      pose
    )

//Parameters:
// kinematics A correctly-configured kinematics object for your drivetrain.
// gyroAngle The current gyro angle.

  /** Creates a new Drive. */
  public Drive(){

  }

  public void setVoltage(Supplier<Double> voltageR, Supplier<Double> voltageL) {
    FRmotor.setVoltage(voltageR.get() * MAX_SPEED);
    FLmotor.setVoltage(voltageL.get() * MAX_SPEED);
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public void setDesiredSetpoint(){

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

    drivePID.calculate();

  }

  @Override
  //closing everything that needs to be closed
  public void close() throws Exception {
    FRmotor.close();
    FLmotor.close();
    MRmotor.close();
    MLmotor.close();
    BRmotor.close();
    BLmotor.close();
    gyro.close();
  }

  //add getFaults() here later

  public List<HardwareFault> getFaults(){
    
  }
}
