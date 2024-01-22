// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private CANSparkMax angleMotor;
  private DutyCycleEncoder angleEncoder;
  private UsbCamera usbCamera;
  private AnalogInput pieceDetector;
  private PIDController angleController;
  public enum STATE {
    STATE1,
    STATE2
  }
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(Constants.IntakeSubsystemConstants.kAngleMotorPort, MotorType.kBrushless);
    angleMotor = new CANSparkMax(Constants.IntakeSubsystemConstants.kIntakeMotorPort, MotorType.kBrushless);
    angleEncoder = new DutyCycleEncoder(Constants.IntakeSubsystemConstants.kAngleEncoderChannel);
    angleEncoder.setPositionOffset(Constants.IntakeSubsystemConstants.kAngleEncoderOffset);
    // usbCamera = new UsbCamera(); //TODO: Research
    pieceDetector = new AnalogInput(Constants.IntakeSubsystemConstants.kPieceDetectorAnalogInputPort);
    angleController = new PIDController(Constants.IntakeSubsystemConstants.kKp, 0, 0);
  }
  public void setAnglePosition(double position){
    angleController.setSetpoint(position);
  }
  public void setMotorMode(STATE state) {
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
  public double getPieceX(){ //Get X value of gamepiece on camera
    return 0;
  }
  public double getPieceY(){ //Get Y value of gamepiece on camera
    return 0;
  }
  public double getPieceA(){ //Get distance of gamepiece from camera
    return 0;
  }

  @Override
  public void periodic() {
    angleMotor.set(angleController.calculate(angleEncoder.getAbsolutePosition()));
  }
}