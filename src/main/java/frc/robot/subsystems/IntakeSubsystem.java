// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
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
  private enum state {
    STATE1,
    STATE2,
    STATE3
  }

  //public static class IntakeSubsystemConstants{
  //  public static final int kIntakeMotorPort = 10;
  //  public static final int kAngleMotorPort = 11;
  //  public static final int kAngleEncoderChannel = 0;
  //  public static final int kPieceDetectorAnalogInputPort = 0;
  //  public static final int kLimitSwitchPort = 1;
  //  public static final double pieceDetector_DetectionThreshold = 2.5;
  //}
  ////These are the constants

  public IntakeSubsystem() {
    intakeMotor = new Spark(Constants.IntakeSubsystemConstants.kIntakeMotorPort);
    angleMotor = new Spark(Constants.IntakeSubsystemConstants.kAngleMotorPort);
    angleEncoder = new DutyCycleEncoder(Constants.IntakeSubsystemConstants.kAngleEncoderChannel); //Using the ctor in the class that gets a channel number
    usbCamera = new UsbCamera(getName(), getName());
    pieceDetector = new AnalogInput(Constants.IntakeSubsystemConstants.kPieceDetectorAnalogInputPort);
    angleController = new PIDController(0.5, 0, 0);
  }
  public void setAnglePosition(double position){
    angleController.setSetpoint(position);
    angleMotor.set(angleController.calculate(angleEncoder.get()));
  }
  public void setMotorMode(State state) { //I'm not sure which state class to import
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

        break;
    }
  }
  public double getAbsoluteAngle(){
    return angleEncoder.get();
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

  public void intakeOutput(double power){
    intakeMotor.set(power);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
