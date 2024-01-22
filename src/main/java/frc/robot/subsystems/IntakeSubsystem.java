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
    STATE2,
    STATE3
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
        // sets motor mode as STATE1
        break;
      case STATE2:
        // sets motor mode as STATE2
        break;
      case STATE3:
        // sets motor mode as STATE3
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
