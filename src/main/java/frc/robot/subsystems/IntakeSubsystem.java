// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private Spark intakeMotor;
  private Spark angleMotor;
  private DutyCycleEncoder angleEncoder;
  private UsbCamera usbCamera;
  private AnalogInput pieceDetector;
  private PIDController angleController;
  public enum state {
    STATE1,
    STATE2
  }
  public IntakeSubsystem() {
    intakeMotor = new Spark(Constants.IntakeSubsystemConstants.kIntakeMotorPort);
    angleMotor = new Spark(Constants.IntakeSubsystemConstants.kAngleMotorPort);
    angleEncoder = new DutyCycleEncoder(Constants.IntakeSubsystemConstants.kAngleEncoderChannel);
    angleEncoder.setPositionOffset(Constants.IntakeSubsystemConstants.kAngleEncoderOffset);
    // usbCamera = new UsbCamera(); //TODO: Research
    pieceDetector = new AnalogInput(Constants.IntakeSubsystemConstants.kPieceDetectorAnalogInputPort);
    angleController = new PIDController(Constants.IntakeSubsystemConstants.kKp, 0, 0);
  }
  public void setAnglePosition(double position){
    angleController.setSetpoint(position);
  }
  public void setMotorMode(state state) { 
    switch (state) {
      case STATE1:
        intakeMotor.set(Constants.IntakeSubsystemConstants.kIntakeMotorIntakeSpeed); //Intake
        break;
      case STATE2:
        intakeMotor.set(Constants.IntakeSubsystemConstants.kIntakeMotorOutputSpeed); //Output
        break;
      default:
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
  public double getPieceX(){
    return 0;
  }
  public double getPieceY(){
    return 0;
  }
  public double getPieceA(){
    return 0;
  }

  @Override
  public void periodic() {
    angleMotor.set(angleController.calculate(angleEncoder.getAbsolutePosition()));
  }
}

// public static class IntakeSubsystemConstants{
//     public static final int kIntakeMotorPort = 10;
//     public static final int kAngleMotorPort = 11;
//     public static final int kAngleEncoderChannel = 0;
//     public static final int kPieceDetectorAnalogInputPort = 0;
//     public static final int kLimitSwitchPort = 1;
//     public static final double pieceDetector_DetectionThreshold = 2.5; //TODO: Find actual value
//     public static final double kAngleEncoderOffset = 0.2; //TODO: Find actual value
//     public static final double kKp = 0.5; //TODO: Find actual value
//     public static final double kIntakeMotorOutputSpeed = 0.75; //TODO: Find Actual Speed needed, and find out which way the intake motor is facing
//     public static final double kIntakeMotorIntakeSpeed = 0.75;
//   }