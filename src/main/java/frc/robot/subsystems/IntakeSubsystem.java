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

  private AnalogInput pieceDetector; // Distance sensor responsible for detecting game piece in the system

  public enum INTAKE_STATE { // Enum representing the 3 states of the intake motor
    collectState,
    outputState,
    restState,
  }

  // Limelight values:
  private LimelightUtil limelight;

  public IntakeSubsystem() {
    this.intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorPort);
    this.pieceDetector = new AnalogInput(IntakeConstants.kPieceDetectorPort);
    this.limelight = new LimelightUtil("limelight-collect");

    this.intakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
     * Getter for the Limelight utility used by the subsystem for vision processing
     * @return Limelight utility object used by the subsystem for vision processing
  */
  public LimelightUtil getLimelight() {
    return this.limelight;
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
        this.intakeMotor.set(0); // Rest
        break;
    }
  }

  /**
   * Checks if there is a game piece in the intake system
   * @return True if piece is inside the intake system
  */
  public boolean hasPiece(){
    return pieceDetector.getVoltage() > 1.1 && pieceDetector.getVoltage() <3.5;
  }

  @Override
  public void periodic() {   ;
      

    LEDController.setStorageState(hasPiece() ? LEDController.StorageStates.HOLDING_PIECE :  LEDController.StorageStates.EMPTY);

    SmartDashboard.putBoolean("Has Piece", hasPiece());
    SmartDashboard.putNumber("Detector Voltage",this.pieceDetector.getVoltage());

    SmartDashboard.putBoolean("Camera has target", this.limelight.cameraHasTarget());
  } 
}