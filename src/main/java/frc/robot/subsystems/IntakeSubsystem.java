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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private CANSparkMax angleMotor;
  private DutyCycleEncoder angleEncoder;
  private AnalogInput pieceDetector;
  private ArmFeedforward armFeedForward;
  public enum STATE {
    collectingState,
    outputtingState,
    restState
  }
  private TrapezoidProfile.State goalState;
  private final TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile angleController;
  private double lastTime = 0;

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry ta = table.getEntry("ta");

  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(Constants.IntakeSubsystemConstants.kAngleMotorPort, MotorType.kBrushless);
    angleMotor = new CANSparkMax(Constants.IntakeSubsystemConstants.kIntakeMotorPort, MotorType.kBrushless);
    angleEncoder = new DutyCycleEncoder(Constants.IntakeSubsystemConstants.kAngleEncoderChannel);
    angleEncoder.setPositionOffset(Constants.IntakeSubsystemConstants.kAngleEncoderOffset);
    
    pieceDetector = new AnalogInput(Constants.IntakeSubsystemConstants.kPieceDetectorAnalogInputPort);
    constraints = new TrapezoidProfile.Constraints(Constants.IntakeSubsystemConstants.kMaxVelocityTrapezoidProfileConstraint, Constants.IntakeSubsystemConstants.kMaxAccelerationTrapezoidProfileConstraint);
  }
  public void setAnglePosition(double position){
    this.goalState = new TrapezoidProfile.State(position, 0);
        angleController = new TrapezoidProfile(constraints);
        lastTime = Timer.getFPGATimestamp();
  }
  public TrapezoidProfile.State getGoalState() {
    return this.goalState;
}
private double calculateAcceleration() {
  return 0.0;//Temporary
}
  private TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(0, 0);
  }
  private double calculateFeedforward(TrapezoidProfile.State setpoint) {
    return 0; //Temporary Return, Input Calculation here
  }
  public void setMotorMode(STATE state) {
    switch (state) {
      case collectingState:
        intakeMotor.set(Constants.IntakeSubsystemConstants.kIntakeMotorIntakeSpeed); //Intake
        break;
      case outputtingState:
        intakeMotor.set(Constants.IntakeSubsystemConstants.kIntakeMotorOutputSpeed); //Output
        break;
      case restState:
         intakeMotor.set(0);
        break;
    }
  }
  public double getAbsoluteAngle(){
    return angleEncoder.getAbsolutePosition();
  } 
  public boolean hasPiece(){
    return pieceDetector.getVoltage() > Constants.IntakeSubsystemConstants.pieceDetector_DetectionThreshold;
  }
  public double getPieceX(){ //Get X value of gamepiece on camera
    return tx.getDouble(0.0);
  }
  public double getPieceY(){ //Get Y value of gamepiece on camera
    return ty.getDouble(0.0);
  }
  public double getPieceA(){ //Get distance of gamepiece from camera
    return ta.getDouble(0.0);
  }


  @Override
  public void periodic() {
    double currentTime = Timer.getFPGATimestamp();
    double timeSinceStart = currentTime - lastTime;
    TrapezoidProfile.State currentState = getCurrentState();
    TrapezoidProfile.State goalState = getGoalState();

    TrapezoidProfile.State setpoint = angleController.calculate(timeSinceStart, currentState, goalState);

    double feedforward = armFeedForward.calculate(setpoint.position, setpoint.velocity, calculateAcceleration());
    angleMotor.set(feedforward);

    double dt = currentTime - lastTime; // Calculate the time since the last periodic call
    lastTime = currentTime;

    
    double currentAcceleration = calculateAcceleration();

    double feedforwardVoltage = armFeedForward.calculate(
        setpoint.position,
        setpoint.velocity,
        currentAcceleration
    );


    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
  
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    if (angleController != null) {
        feedforward = calculateFeedforward(setpoint);
    }
} }