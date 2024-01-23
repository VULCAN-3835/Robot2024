// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModule {
    private TalonFX driveMotor; // Falcon 500 motor responsible for driving the module
    private TalonFX steerMotor; // Falcon 500 motor responsible for steering the module
    private CANcoder absEncoder; // Absolute encoder responsible for keeping track of module position

    private final double absoluteEncoderOffset;

    private SwerveModuleState state;

    private StatusSignal<Double> m_drivePosition; // Drive position supplier
    private StatusSignal<Double> m_driveVelocity; // Drive velocity supplier
    private StatusSignal<Double> m_steerPosition; // Steer position supplier
    private StatusSignal<Double> m_steerVelocity; // Steer velocity supplier

    private BaseStatusSignal[] m_signals; // Array of the suppliers

    private SwerveModulePosition swervePosition = new SwerveModulePosition();

    private PositionVoltage angleSetter = new PositionVoltage(0);

    public SwerveModule(int driveMotorID, int steerMotorID, int absEncoderID, 
    boolean driveMotorReversed,double absoluteEncoderOffset) {
        this.driveMotor = new TalonFX(driveMotorID);
        this.steerMotor = new TalonFX(steerMotorID);
        this.absEncoder = new CANcoder(absEncoderID);

        this.absoluteEncoderOffset = absoluteEncoderOffset;

        this.driveMotor.setInverted(driveMotorReversed);

        this.driveMotor.setNeutralMode(NeutralModeValue.Brake);
        this.steerMotor.setNeutralMode(NeutralModeValue.Brake);

        configEnc();
        configDriveMotor();
        configSteerMotor(absEncoderID);

        m_drivePosition = driveMotor.getPosition();
        m_driveVelocity = driveMotor.getVelocity();
        m_steerPosition = absEncoder.getPosition();
        m_steerVelocity = absEncoder.getVelocity();

        m_signals = new BaseStatusSignal[4];
        m_signals[0] = m_drivePosition;
        m_signals[1] = m_driveVelocity;
        m_signals[2] = m_steerPosition;
        m_signals[3] = m_steerVelocity;

        angleSetter.FeedForward = Constants.ModuleConstants.kFeedforwardGainSteer;
    }

    private void configEnc() { // TODO: Check all constants
        CANcoderConfiguration canConfigs = new CANcoderConfiguration();

        canConfigs.MagnetSensor.MagnetOffset = this.absoluteEncoderOffset;

        this.absEncoder.getConfigurator().apply(canConfigs);
    }

    private void configSteerMotor(int absEncoderID) { // TODO: Check all constants
        TalonFXConfiguration steerConfigs = new TalonFXConfiguration();

        steerConfigs.Slot0 = Constants.ModuleConstants.getSteerMotorGains();

        steerConfigs.Feedback.FeedbackRemoteSensorID = absEncoderID;
        steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        steerConfigs.Feedback.RotorToSensorRatio = Constants.ModuleConstants.kSteerMotorGearRatio;

        steerConfigs.ClosedLoopGeneral.ContinuousWrap = true;

        this.steerMotor.getConfigurator().apply(steerConfigs);
    }

    private void configDriveMotor() { // TODO: Do drive motor configs
        
    }
    public double getModuleAngle() {
        m_steerPosition.refresh();
        return m_steerPosition.getValue();
    }
    public double getModuleAngleError() {
        var error = this.steerMotor.getClosedLoopError();
        error.refresh();
        return error.getValue();
    }
    public double getModuleClosedLoopOutput() {
        var output = this.steerMotor.getClosedLoopOutput();
        output.refresh();
        return output.getValue();
    }
    


    public SwerveModulePosition getPosition(boolean refresh) {
        if (refresh) {
            m_drivePosition.refresh();
            m_driveVelocity.refresh();
            m_steerPosition.refresh();
            m_steerVelocity.refresh();
        }

        double drive_rot =
            BaseStatusSignal.getLatencyCompensatedValue(m_drivePosition, m_driveVelocity);
        double angle_rot =
            BaseStatusSignal.getLatencyCompensatedValue(m_steerPosition, m_steerVelocity);

        swervePosition.distanceMeters = drive_rot; // TODO: Into meters
        swervePosition.angle = Rotation2d.fromRotations(angle_rot);

        return swervePosition;
    }

    public void set(SwerveModuleState state) {
        this.state = state;

        m_steerPosition.refresh();
        m_steerVelocity.refresh();

        double angle_rot =
            BaseStatusSignal.getLatencyCompensatedValue(m_steerPosition, m_steerVelocity);

        SwerveModuleState optimized = SwerveModuleState.optimize(this.state, Rotation2d.fromRotations(angle_rot));
        if (Math.abs(optimized.speedMetersPerSecond) < 0.001) {
            stopModules();
            return;
        }

        this.angleSetter.FeedForward = getModuleAngleError()>=0?Constants.ModuleConstants.kFeedforwardGainSteer:-Constants.ModuleConstants.kFeedforwardGainSteer;
        double angleToSetDeg = optimized.angle.getRotations();
        this.steerMotor.setControl(this.angleSetter.withPosition(angleToSetDeg));
        driveMotor.set(optimized.speedMetersPerSecond);
    }

    public void stopModules() {
        this.driveMotor.set(0);
        this.steerMotor.set(0);
    }
}