package org.sciborgs1155.robot.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import org.sciborgs1155.lib.constants.*;
import org.sciborgs1155.robot.Ports.Drive;

public class DriveMotor {

    public DriveMotor(){
      CANSparkMax Rmotor = SparkUtils.create(
        Drive.RdrivePort,
            s -> {
              s.setInverted(false);
              s.setIdleMode(IdleMode.kBrake);
              s.setOpenLoopRampRate(0);
              s.setSmartCurrentLimit(50);
            });
    
      CANSparkMax Lmotor = SparkUtils.create(
          Drive.LdrivePort,
              s -> {
                s.setInverted(false);
                s.setIdleMode(IdleMode.kBrake);
                s.setOpenLoopRampRate(0);
                s.setSmartCurrentLimit(50);
              });
    }
        
}
