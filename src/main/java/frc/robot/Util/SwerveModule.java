// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

/** Add your docs here. */
public class SwerveModule {
    private TalonFX driveMotor; // Falcon 500 motor responsible for driving the module
    private TalonFX steerMotor; // Falcon 500 motor responsible for steering the module
    private CANcoder absEncoder; // Absolute encoder responsible for keeping track of module position

    private final double absoluteEncoderOffset;
    private final boolean absoluteEncoderReversed;

    public SwerveModule(int driveMotorID, int steerMotorID, int absEncoderID, boolean absoluteEncoderReversed,
     boolean steerMotorReversed, boolean driveMotorReversed,double absoluteEncoderOffset) {
        this.driveMotor = new TalonFX(driveMotorID);
        this.steerMotor = new TalonFX(steerMotorID);
        this.absEncoder = new CANcoder(absEncoderID);

        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        this.driveMotor.setInverted(driveMotorReversed);
        this.steerMotor.setInverted(steerMotorReversed);

        this.driveMotor.setNeutralMode(NeutralModeValue.Brake);
        this.steerMotor.setNeutralMode(NeutralModeValue.Brake);

        configMotors();
        configEnc();
    }
    private void configEnc() {
        CANcoderConfigurator canConfig = this.absEncoder.getConfigurator();
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();

        magnetConfig.MagnetOffset = this.absoluteEncoderOffset;
        magnetConfig.SensorDirection = this.absoluteEncoderReversed?
        SensorDirectionValue.CounterClockwise_Positive:
        SensorDirectionValue.Clockwise_Positive;
        magnetConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        canConfig.apply(magnetConfig);
    }
    private void configMotors() {
        TalonFXConfigurator driveConfig = this.driveMotor.getConfigurator();
        TalonFXConfigurator steerConfig = this.steerMotor.getConfigurator();

        FeedbackConfigs driveFeedbackConfigs = new FeedbackConfigs();
        FeedbackConfigs steerFeedbackConfigs = new FeedbackConfigs();

        driveFeedbackConfigs.RotorToSensorRatio = 1;
        driveFeedbackConfigs.SensorToMechanismRatio = 1;

        steerFeedbackConfigs.RotorToSensorRatio = 1;
        steerFeedbackConfigs.SensorToMechanismRatio = 1;

        driveConfig.apply(driveFeedbackConfigs);
        steerConfig.apply(steerFeedbackConfigs);
    }
    public double getModuleAngle() {
        return this.absEncoder.getPosition().getValue()*360;
    }
}