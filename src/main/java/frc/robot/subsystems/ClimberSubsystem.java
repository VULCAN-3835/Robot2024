// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;;


public class ClimberSubsystem extends SubsystemBase {
  private TalonFX climberMotorRight;
  private TalonFX climberMotorLeft;
  private DigitalInput limitSwitchRight;
  private DigitalInput limitSwitchLeft;

  private StatusSignal<Double> m_leftPosition;
  private StatusSignal<Double> m_leftVelocity;
  private StatusSignal<Double> m_rightPosition;
  private StatusSignal<Double> m_rightVelocity;

  MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

  private XboxController xboxController;
  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(XboxController xboxController) {
    this.climberMotorRight=new TalonFX(Constants.ClimberConstants.kRightMotorPort);
    this.climberMotorLeft=new TalonFX(Constants.ClimberConstants.kLeftMotorPort);

    this.limitSwitchRight=new DigitalInput(Constants.ClimberConstants.kRightSwitchPort);
    this.limitSwitchRight=new DigitalInput(Constants.ClimberConstants.kLeftSwitchPort);

    this.climberMotorLeft.setInverted(Constants.ClimberConstants.kLeftInverted);
    this.climberMotorRight.setInverted(Constants.ClimberConstants.kRightInverted);
      
    TalonFXConfiguration configuration = new TalonFXConfiguration();

    // Feedback sensor returns rotations of axis
    configuration.Feedback.SensorToMechanismRatio = Constants.ClimberConstants.kMotorRatio;

    configuration.Slot0 = Constants.ClimberConstants.getElevatorSlot();
    configuration.MotionMagic.MotionMagicAcceleration = Constants.ClimberConstants.kElevatorAcceleration;
    configuration.MotionMagic.MotionMagicCruiseVelocity = Constants.ClimberConstants.kElevatorMaxCruiseVelocity;
    
    this.climberMotorLeft.getConfigurator().apply(configuration);
    this.climberMotorRight.getConfigurator().apply(configuration);

    this.m_leftPosition = this.climberMotorLeft.getPosition();
    this.m_leftVelocity = this.climberMotorLeft.getVelocity();
    this.m_rightPosition = this.climberMotorRight.getPosition();
    this.m_rightVelocity = this.climberMotorRight.getVelocity();

    this.xboxController = xboxController;
  }

  public boolean getRightLimitSwitch(){
    return this.limitSwitchRight.get();
  }

   public boolean getLeftLimitSwitch(){
    return this.limitSwitchLeft.get();
  }

  public double getLeftRotations() {
    this.m_leftPosition.refresh();
    this.m_leftVelocity.refresh();

    double pos = StatusSignal.getLatencyCompensatedValue(m_leftPosition, m_leftVelocity);

    return pos;
  }

  public double getRightRotations() {
    this.m_rightPosition.refresh();
    this.m_rightVelocity.refresh();

    double pos = StatusSignal.getLatencyCompensatedValue(m_rightPosition, m_rightVelocity);

    return pos;
  }

  public void setClimberPosition(double pos){
    this.climberMotorLeft.setControl(this.motionMagicVoltage.withPosition(pos));
    this.climberMotorRight.setControl(this.motionMagicVoltage.withPosition(pos));
  }

  @Override
  public void periodic() {
    double power = (-xboxController.getRightY())*0.3;

    this.climberMotorLeft.set(power);
    this.climberMotorRight.set(power);

  }
}
