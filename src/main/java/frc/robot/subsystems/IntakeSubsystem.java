// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  
  private CANSparkMax intakeMotor; // Motor responsible for intake and output of game pieces
  private CANSparkMax angleMotor; // Motor responsible for the angle of the arm
  private DutyCycleEncoder angleEncoder; // Encoder responsible for keeping the absolute position of the arm

  private AnalogInput pieceDetector; // Distance sensor responsible for detecting game piece in the system
  private DigitalInput closedLimitSwitch; // Saftey limit switch for closed arm
  private DigitalInput openLimitSwitch; // Saftey limit switch for open arm
  
  private ArmFeedforward armFeedForward; // Feed forward values for the arm

  private double goalSetpoint; // Setpoint for the arm position
  private double currentPosition; // The current position of the arm

  private ProfiledPIDController armPositionController; // Closed Loop controller for arm position

  public enum STATE { // Enum representing the 3 states of the intake motor
    collectState,
    outputState,
    restState
  }

  private XboxController xboxController;

  // Limelight values:
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry ta = table.getEntry("ta");

  public IntakeSubsystem(XboxController xboxController) {
    this.xboxController = xboxController;

    this.intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
    this.angleMotor = new CANSparkMax(IntakeConstants.kAngleMotorPort, MotorType.kBrushless);

    this.angleEncoder = new DutyCycleEncoder(IntakeConstants.kAngleEncoderPort);
    this.angleEncoder.setPositionOffset(IntakeConstants.kAngleEncoderOffset);

    this.openLimitSwitch = new DigitalInput(IntakeConstants.kOpenLimitSwitchPort);
    this.closedLimitSwitch = new DigitalInput(IntakeConstants.kClosedLimitSwitchPort);
    
    this.pieceDetector = new AnalogInput(IntakeConstants.kPieceDetectorPort);
    
    this.armPositionController = new ProfiledPIDController(IntakeConstants.kP, 0, 0,
     IntakeConstants.kConstraints);

    this.goalSetpoint = IntakeConstants.kClosedAngle;
    this.armPositionController.setGoal(this.goalSetpoint);

    this.angleMotor.setIdleMode(IdleMode.kBrake);
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
  public double getCurrentPosition() {
    this.currentPosition = this.angleEncoder.getAbsolutePosition()-this.angleEncoder.getPositionOffset();
    return this.currentPosition;
  }

  /**
   * Sets the intake motor's intake to given state
   * @param state The new state of the intake motor
  */
  public void setMotorMode(STATE state) {
    switch (state) {
      case collectState:
        this.intakeMotor.set(Constants.IntakeConstants.kIntakeMotorIntakeSpeed); //Intake
        break;
      case outputState:
        this.intakeMotor.set(Constants.IntakeConstants.kIntakeMotorOutputSpeed); //Output
        break;
      case restState:
        this.intakeMotor.set(0);
        break;
    }
  }

  /**
   * Checks if there is a game piece in the intake system
   * @return True if piece is inside the intake system
  */
  public boolean hasPiece(){
    return pieceDetector.getVoltage() > Constants.IntakeConstants.kPieceDetectorDetectionThreshold;
  }

  /**
   * Finds the X value of the limelight from detected game piece
   * @return X axis value from the game piece
  */
  public double getPieceX(){ 
    return tx.getDouble(0.0);
  }

  /**
   * Finds the Y value of the limelight from detected game piece
   * @return Y axis value from the game piece
  */
  public double getPieceY(){ 
    return ty.getDouble(0.0);
  }

  /**
   * Finds the A value of the limelight from detected game piece
   * @return The area the game piece takes in the limelight's frame
  */
  public double getPieceA(){ 
    return ta.getDouble(0.0);
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
    double output = this.armPositionController.calculate(getCurrentPosition());

    // double output = (xboxController.getLeftTriggerAxis()-xboxController.getRightTriggerAxis())*0.3;

    // Creates limit for the output using limit switches
    if (isOpen() && output < 0)
      output = 0;
    if (isClosed() && output > 0)
      output = 0;
      
    // Doesn't let setpoints pass sensor limits
    if (this.goalSetpoint<IntakeConstants.kOpenAngle)
      this.goalSetpoint = IntakeConstants.kOpenAngle;
    if (this.goalSetpoint>IntakeConstants.kClosedAngle)
      this.goalSetpoint = IntakeConstants.kClosedAngle;

    // Applies output to motor
    this.angleMotor.set(output);

    SmartDashboard.putNumber("Output", output);
    SmartDashboard.putNumber("setpoint", this.goalSetpoint);
    SmartDashboard.putNumber("Current", getCurrentPosition());
    SmartDashboard.putNumber("Error", this.goalSetpoint-getCurrentPosition());

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
  
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    SmartDashboard.putBoolean("Open", isOpen());
    SmartDashboard.putBoolean("Closed", isClosed());
  } 
}