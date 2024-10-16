// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Util.LEDController;
import frc.robot.Util.LimelightUtil;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intakeMotor; // Motor responsible for intake and output of game pieces
  private CANSparkMax angleMotor; // Motor responsible for the angle of the arm
  private DutyCycleEncoder angleEncoder; // Encoder responsible for keeping the absolute position of the arm

  private AnalogInput pieceDetector; // Distance sensor responsible for detecting game piece in the system
  private DigitalInput closedLimitSwitch; // Saftey limit switch for closed arm
  private DigitalInput openLimitSwitch; // Saftey limit switch for open arm

  private double goalSetpoint; // Setpoint for the arm position
  private double currentPosition; // The current position of the arm

  private ProfiledPIDController armPositionController; // Closed Loop controller for arm position

  public enum INTAKE_STATE { // Enum representing the 4 states of the intake motor
    collectState,
    outputState,
    restState,
    ampState
  }

  // Limelight values:
  private LimelightUtil limelight;

  public IntakeSubsystem() {
    this.intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorPort);
    this.angleMotor = new CANSparkMax(IntakeConstants.kAngleMotorPort, MotorType.kBrushless);

    this.angleEncoder = new DutyCycleEncoder(IntakeConstants.kAngleEncoderPort);
    this.angleEncoder.setPositionOffset(IntakeConstants.kAngleEncoderOffset);

    this.openLimitSwitch = new DigitalInput(IntakeConstants.kOpenLimitSwitchPort);
    this.closedLimitSwitch = new DigitalInput(IntakeConstants.kClosedLimitSwitchPort);
    
    this.pieceDetector = new AnalogInput(IntakeConstants.kPieceDetectorPort);
    
    this.armPositionController = new ProfiledPIDController(IntakeConstants.kP, 0, 0,
     IntakeConstants.kConstraints);
    
    this.limelight = new LimelightUtil("limelight-collect");

    this.goalSetpoint = IntakeConstants.kClosedRotations;
    this.armPositionController.setGoal(this.goalSetpoint);

    this.angleMotor.setIdleMode(IdleMode.kBrake);
    this.intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    this.angleMotor.setInverted(false);

    this.armPositionController.setTolerance(0.007);
  }

  /**
     * Getter for the Limelight utility used by the subsystem for vision processing
     * @return Limelight utility object used by the subsystem for vision processing
  */
  public LimelightUtil getLimelight() {
    return this.limelight;
  }

  /**
     * Getter for the arm setpoint (goal)
     * @return The setpoint of the arm in rotations (1 = 360*)
  */
  public boolean getArmAtSetpoint() {
    return this.armPositionController.atSetpoint();
  }

  /**
   * Sets the arm angle controller's goal to given position
   * @param position The new position for the arm
  */
  public void setRotationPosition(double position){
    this.goalSetpoint = position;
    this.armPositionController.setGoal(this.goalSetpoint);
  }

  /**
   * Return's the current position of the arm 
   * @return The current position of the arm
  */
  private double getCurrentPosition() {
    this.currentPosition = (this.angleEncoder.getAbsolutePosition()-this.angleEncoder.getPositionOffset());
    this.currentPosition = normalizePosition(this.currentPosition);
    return this.currentPosition;
  }

  /**
    * Normalizes a position value to be within the range [0, 1).
    * This method takes a value, adds 1 to it, and then uses the modulo operation 
    * to ensure the result is within the specified range. This is useful for wrapping 
    * values around when they exceed the range of [0, 1).
    * @param value The position value to be normalized.
    * @return The normalized position value within the range [0, 1).
  */
  public double normalizePosition(double value) {
    return (value+1)%1;
  }

  /**
   * Sets the intake motor's intake to given state
   * @param state The new state of the intake motor
  */
  public void setMotorMode(INTAKE_STATE state) {
    switch (state) {
      case collectState:
        this.intakeMotor.set(IntakeConstants.kMotorIntakePower); //Intake
        break;
      case outputState:
        this.intakeMotor.set(IntakeConstants.kMotorOutputPower); //Output
        break;
      case restState:
        this.intakeMotor.set(0);
        break;
      case ampState:
        this.intakeMotor.set(IntakeConstants.kAmpOutputPower);
    }
  }

  /**
   * Checks if there is a game piece in the intake system
   * @return True if piece is inside the intake system
  */
  public boolean hasPiece(){
    return pieceDetector.getVoltage() > IntakeConstants.kPieceDetectorDetectionThreshold && pieceDetector.getVoltage() <2.1;
  }


  /**
   * Checks if arm is closed
   * @return True if arm is at closed limit switch
  */
  public boolean isClosed() {
    return this.closedLimitSwitch.get();
  }

  /**
   * Checks if arm is open
   * @return True if arm is at open limit switch
  */
  public boolean isOpen() {
    return this.openLimitSwitch.get();
  }

  @Override
  public void periodic() {   
    // Calculates the output for moving the arm 
    double output = -this.armPositionController.calculate(getCurrentPosition());

    // Creates limit for the output using limit switches
    if (isOpen() && output > 0)
      output = 0;
    if (isClosed() && output < 0)
      output = 0;
      
    // Doesn't let setpoints pass sensor limits
    if (this.goalSetpoint<IntakeConstants.kOpenRotations)
      this.goalSetpoint = IntakeConstants.kOpenRotations;
    if (this.goalSetpoint>IntakeConstants.kClosedRotations)
      this.goalSetpoint = IntakeConstants.kClosedRotations;

    if (output > 0.825)
      output = 0.825;
    if (output < -0.825)
      output = -0.825;

    if (getCurrentPosition() == 0.25)
      output = 0;
    // Applies output to motor
    this.angleMotor.set(output);

    LEDController.setStorageState(hasPiece() ? LEDController.StorageStates.HOLDING_PIECE :  LEDController.StorageStates.EMPTY);


    SmartDashboard.putNumber("Intake Output", output);
    SmartDashboard.putNumber("Intake setpoint", this.goalSetpoint);
    SmartDashboard.putNumber("Intake Current", getCurrentPosition());
    SmartDashboard.putNumber("Intake Error", this.goalSetpoint-getCurrentPosition());

    SmartDashboard.putBoolean("Intake Open", isOpen());
    SmartDashboard.putBoolean("Intake Closed", isClosed());

    SmartDashboard.putBoolean("Has Piece", hasPiece());
    SmartDashboard.putNumber("Detector Voltage",this.pieceDetector.getVoltage());

    SmartDashboard.putBoolean("Camera has target", this.limelight.cameraHasTarget());

    SmartDashboard.putBoolean("At Setpoint", getArmAtSetpoint());
  } 
}