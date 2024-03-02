// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */
public class SwerveModule {
    private TalonFX driveMotor; // Falcon 500 motor responsible for driving the module
    private TalonFX steerMotor; // Falcon 500 motor responsible for steering the module
    private CANcoder absEncoder; // Absolute encoder responsible for keeping track of module position

    private final double absoluteEncoderOffset; // The offset of the absolute encoder from it's true zero

    private SwerveModuleState state; // The state the module aims to be

    // Phoenix6 suppliers for the different feedback values
    private StatusSignal<Double> m_drivePosition; // Drive position supplier
    private StatusSignal<Double> m_driveVelocity; // Drive velocity supplier
    private StatusSignal<Double> m_driveVoltage; // Drive voltage supplier
    private StatusSignal<Double> m_steerPosition; // Steer position supplier
    private StatusSignal<Double> m_steerVelocity; // Steer velocity supplier

    private SwerveModulePosition swervePosition = new SwerveModulePosition(); // The position of the module (Releveant for poseEstimator)

    private PositionVoltage angleController = new PositionVoltage(0); // The closed loop controller for module angle
    private VelocityVoltage velocityController = new VelocityVoltage(0);

    private SimpleMotorFeedforward driveFeedForward;

    public SwerveModule(int driveMotorID, int steerMotorID, int absEncoderID, 
    boolean driveMotorInverted,double absoluteEncoderOffset, SimpleMotorFeedforward ff) {
        // Motor controllers + Sensors initialization:
        this.driveMotor = new TalonFX(driveMotorID);
        this.steerMotor = new TalonFX(steerMotorID);
        this.absEncoder = new CANcoder(absEncoderID);

        // Offset storing:
        this.absoluteEncoderOffset = absoluteEncoderOffset;

        // Advannced configs:
        configEnc();
        configSteerMotor(absEncoderID);
        configDriveMotor(driveMotorInverted);

        // Storing the signals (suppliers) of the different feedback sensors
        this.m_drivePosition = this.driveMotor.getPosition();
        this.m_driveVelocity = this.driveMotor.getVelocity();
        this.m_driveVoltage = this.driveMotor.getMotorVoltage();
        this.m_steerPosition = this.absEncoder.getPosition();
        this.m_steerVelocity = this.absEncoder.getVelocity();

        // Config for the angle closed loop control feedforward value to overcome friction
        this.angleController.FeedForward = Constants.ModuleConstants.kFeedforwardGainSteer;

        this.driveFeedForward = ff;
    }

    /**
     * Configures encoder offset
    */
    private void configEnc() { 
        CANcoderConfiguration canConfigs = new CANcoderConfiguration();

        canConfigs.MagnetSensor.MagnetOffset = this.absoluteEncoderOffset; // Sets the offset of the Cancoder

        this.absEncoder.getConfigurator().apply(canConfigs);
    }

    /**
     * Configures the steer motor's feedback sennsor, source, rotor to sensor ratio and countinous readings
     *
     * @param absEncoderID The id of the absolut encoder to set as the new feedback sensor
    */
    private void configSteerMotor(int absEncoderID) { 
        TalonFXConfiguration steerConfigs = new TalonFXConfiguration();

        steerConfigs.Slot0 = Constants.ModuleConstants.getSteerMotorGains(); // Sets constant closed loop values

        steerConfigs.Feedback.FeedbackRemoteSensorID = absEncoderID; // Changes the feedback sensor of the Steer motor into Cancoder
        steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder; // Sets the source into can coder
        steerConfigs.Feedback.RotorToSensorRatio = Constants.ModuleConstants.kSteerMotorGearRatio; // Sets a ration between motor and sensor

        steerConfigs.ClosedLoopGeneral.ContinuousWrap = true; // Changes closed loop into continues values

        steerConfigs.CurrentLimits.SupplyCurrentLimit = ModuleConstants.kSteerCurrentLimit;
        steerConfigs.CurrentLimits.SupplyCurrentThreshold = ModuleConstants.kSteerCurrentThreshold;
        steerConfigs.CurrentLimits.SupplyTimeThreshold = ModuleConstants.kSteerCurrentThresholdTime;
        steerConfigs.CurrentLimits.SupplyCurrentLimitEnable = ModuleConstants.kSteerEnableCurrentLimit;

        steerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        this.steerMotor.getConfigurator().apply(steerConfigs);
    }

    private void configDriveMotor(boolean inverted) {
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();

        driveConfigs.Feedback.SensorToMechanismRatio = ModuleConstants.kDriveMotorGearRatio;

        driveConfigs.CurrentLimits.SupplyCurrentLimit = ModuleConstants.kDriveCurrentLimit;
        driveConfigs.CurrentLimits.SupplyCurrentThreshold = ModuleConstants.kDriveCurrentThreshold;
        driveConfigs.CurrentLimits.SupplyTimeThreshold = ModuleConstants.kDriveCurrentThresholdTime;
        driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = ModuleConstants.kDriveEnableCurrentLimit;

        driveConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfigs.MotorOutput.Inverted = inverted?InvertedValue.Clockwise_Positive:InvertedValue.CounterClockwise_Positive;

        this.driveMotor.getConfigurator().apply(driveConfigs);
        this.driveMotor.getConfigurator().setPosition(0);
    }

    /**
     * Returns the module's angle closed loop controller error in rotations (i.e 1 is a full rotation, 0.25 a qurter of a rotation or 90 degrees)
     * @return The module's angle closed loop controller error in rotations
    */
    public double getModuleAngleError() {
        var error = this.steerMotor.getClosedLoopError();
        error.refresh();
        return error.getValue();
    }
    
    /**
     * Returns the module's angle closed loop controller output to the motor in power precentage (i.e between -1 to 1 where 1 is full 
     * power to the positive side of the motor and -1 is full power to the negative side of the motor)
     * @return the module's angle closed loop controller output to the motor
    */
    public double getModuleClosedLoopOutput() {
        var output = this.steerMotor.getClosedLoopOutput();
        output.refresh();
        return output.getValue();
    }

    public double getModuleDriveOutput() {
        var output = this.driveMotor.getClosedLoopOutput();
        output.refresh();
        return output.getValue();
    }

    /**
     * Returns the module's position using the feedback sensors with latency compensation
     * @return the module's current position
    */
    public SwerveModulePosition getPosition() {
        m_drivePosition.refresh();
        m_driveVelocity.refresh();
        m_steerPosition.refresh();
        m_steerVelocity.refresh();

        double drive_rot =
            BaseStatusSignal.getLatencyCompensatedValue(m_drivePosition, m_driveVelocity);
        double angle_rot =
            BaseStatusSignal.getLatencyCompensatedValue(m_steerPosition, m_steerVelocity);

        swervePosition.distanceMeters = Conversions.rotationsToMeters(drive_rot, ModuleConstants.kWheelCircumference); // Current module's drive position
        swervePosition.angle = Rotation2d.fromRotations(angle_rot); // Current module's angle in Rotation2d

        return swervePosition;
    }

    /**
     * Sets the motor's closed loop controllers setpoints to a certain module state
     * @param state The target state for the module
    */
    public void set(SwerveModuleState state) {
        this.state = state;

        // Refreshes status signals:
        m_steerPosition.refresh();
        m_steerVelocity.refresh();

        // Calculates latency compensated module's angle in rotations
        double angle_rot =
            BaseStatusSignal.getLatencyCompensatedValue(m_steerPosition, m_steerVelocity);


        // Optimizing module's desired state
        SwerveModuleState optimized = SwerveModuleState.optimize(this.state, Rotation2d.fromRotations(angle_rot));

        // To prevent the wheels from resetting back into angle 0 when not given any input
        if (Math.abs(optimized.speedMetersPerSecond) < Constants.ModuleConstants.kModuleAngleDeadband) {
            stopModule();
            return;
        }

        // Change feedforward value based on negative / positive error
        this.angleController.FeedForward = getModuleAngleError()>=0?Constants.ModuleConstants.kFeedforwardGainSteer:-Constants.ModuleConstants.kFeedforwardGainSteer;

        // The desired module's angle in rotations
        double angleToSetDeg = optimized.angle.getRotations();

        this.steerMotor.setControl(this.angleController.withPosition(angleToSetDeg));
        setSpeed(optimized);
                // this.driveMotor.set(optimized.speedMetersPerSecond);

    }

    public void setMotorVoltage(double voltage) {
        this.steerMotor.setControl(this.angleController.withPosition(0));
        this.driveMotor.setVoltage(voltage);
    }

    private void setSpeed(SwerveModuleState desiredState) {
        this.velocityController.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, ModuleConstants.kWheelCircumference);
        this.velocityController.FeedForward = this.driveFeedForward.calculate(desiredState.speedMetersPerSecond);
        this.driveMotor.setControl(this.velocityController);
    }

    /**
     * Stops the module's motors
    */
    public void stopModule() {
        this.driveMotor.set(0);
        this.steerMotor.set(0);
    }

    public double getVelocity() {
        this.m_driveVelocity.refresh();
        return Conversions.RPSToMPS(this.m_driveVelocity.getValue(), ModuleConstants.kWheelCircumference);
    }

    public double getVoltage() {
        this.m_driveVoltage.refresh();
        return m_driveVoltage.getValue();
    }
}
