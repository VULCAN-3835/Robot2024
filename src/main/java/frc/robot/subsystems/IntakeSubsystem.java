// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
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
  
  private CANSparkMax intakeMotor;
  private CANSparkMax angleMotor;

  private DutyCycleEncoder angleEncoder;
  private AnalogInput pieceDetector;
  private DigitalInput closedLimitSwitch;
  private DigitalInput openLimitSwitch;
  
  private ArmFeedforward armFeedForward;

  private double goalSetpoint;
  private double currentPosition;
  private final TrapezoidProfile.Constraints constraints;

  private ProfiledPIDController angleController;
  private XboxController xboxController; // Temp for testing

  public enum STATE {
    collectState,
    outputState,
    restState
  }

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry ta = table.getEntry("ta");

  public IntakeSubsystem(XboxController xboxController) {
    this.intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);

    this.angleMotor = new CANSparkMax(IntakeConstants.kAngleMotorPort, MotorType.kBrushless);

    this.angleEncoder = new DutyCycleEncoder(IntakeConstants.kAngleEncoderPort);
    this.angleEncoder.setPositionOffset(IntakeConstants.kAngleEncoderOffset);

    this.openLimitSwitch = new DigitalInput(IntakeConstants.kOpenLimitSwitchPort);
    this.closedLimitSwitch = new DigitalInput(IntakeConstants.kClosedLimitSwitchPort);
    
    this.pieceDetector = new AnalogInput(IntakeConstants.kPieceDetectorPort);

    this.constraints = new TrapezoidProfile.Constraints(IntakeConstants.kMaxVelocityRadPerSec,
     IntakeConstants.kMaxAccelerationRadPerSecSquared);
    
    this.angleController = new ProfiledPIDController(IntakeConstants.kP, 0, 0, constraints);

    this.xboxController = xboxController;
  }

  public void setAnglePosition(double position){
    position = position / 360; // Into rotations
    this.goalSetpoint = position;
    this.angleController.setGoal(this.goalSetpoint);
  }

  public double getGoalPosition() {
    return this.goalSetpoint;
  }

  public double getCurrentPosition() {
    this.currentPosition = getAbsoluteAngle();
    return this.currentPosition;
  }

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

  public double getAbsoluteAngle(){
    return angleEncoder.getAbsolutePosition();
  } 

  public boolean hasPiece(){
    return pieceDetector.getVoltage() > Constants.IntakeConstants.kPieceDetectorDetectionThreshold;
  }

  public double getPieceX(){ 
    return tx.getDouble(0.0);
  }

  public double getPieceY(){ 
    return ty.getDouble(0.0);
  }

  public double getPieceA(){ 
    return ta.getDouble(0.0);
  }

  @Override
  public void periodic() {    
    this.angleMotor.set((this.xboxController.getLeftTriggerAxis() - this.xboxController.getRightTriggerAxis())*0.3);
    
    if (this.xboxController.getLeftBumper())
      this.intakeMotor.set(0.6);
    else if (this.xboxController.getRightBumper())
      this.intakeMotor.set(-0.8);
    else
      this.intakeMotor.set(0);


    double output = this.angleController.calculate(getCurrentPosition());

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
  
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  } 
}