// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class SwerveModule {
    private TalonFX driveMotor; // Falcon 500 motor responsible for driving the module
    private TalonFX steerMotor; // Falcon 500 motor responsible for steering the module
    private CANcoder absEncoder; // Absolute encoder responsible for keeping track of module position

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffset;

    public SwerveModule(int driveMotorID, int steerMotorID, int absEncoderID, boolean absoluteEncoderReversed,
     boolean steerMotorReversed, boolean driveMotorReversed,double absoluteEncoderOffset) {
        this.driveMotor = new TalonFX(driveMotorID);
        this.steerMotor = new TalonFX(steerMotorID);
        this.absEncoder = new CANcoder(absEncoderID);

        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoderOffset = absoluteEncoderOffset;

        this.driveMotor.setInverted(driveMotorReversed);
        this.steerMotor.setInverted(steerMotorReversed);

        this.driveMotor.setNeutralMode(NeutralModeValue.Brake);
        this.steerMotor.setNeutralMode(NeutralModeValue.Brake);

        this.driveMotor.getVelocity();
        

    }
}
