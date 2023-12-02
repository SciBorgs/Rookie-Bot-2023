package org.sciborgs1155.robot.drive;

import org.sciborgs1155.lib.constants.SparkUtils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

public class StandardDriveMotor {
    public StandardDriveMotor() {};

    public CANSparkMax create(int port){
        final CANSparkMax motor =
        SparkUtils.create(
            port,
            s -> {
                s.setInverted(false);
                s.setIdleMode(IdleMode.kBrake);
                s.setOpenLoopRampRate(0);
                s.setSmartCurrentLimit(50);
            });
            
        return motor;
    }
}
