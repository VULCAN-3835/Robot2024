// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.LEDController;

public class ClimberSubsystem extends SubsystemBase {
  private TalonFX climberMotorRight;
  private TalonFX climberMotorLeft;
  private DigitalInput limitSwitchRight;
  private DigitalInput limitSwitchLeft;

  private boolean ledTrigger;
  

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
    this.limitSwitchLeft=new DigitalInput(Constants.ClimberConstants.kLeftSwitchPort);
      
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    
    // Feedback sensor returns rotations of axis
    configuration.Feedback.SensorToMechanismRatio = Constants.ClimberConstants.kMotorRatio;

    configuration.Slot0 = Constants.ClimberConstants.getElevatorSlot();
    configuration.MotionMagic.MotionMagicAcceleration = Constants.ClimberConstants.kElevatorAcceleration;
    configuration.MotionMagic.MotionMagicCruiseVelocity = Constants.ClimberConstants.kElevatorMaxCruiseVelocity;

    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    this.climberMotorLeft.getConfigurator().apply(configuration);
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    this.climberMotorRight.getConfigurator().apply(configuration);

    this.m_leftPosition = this.climberMotorLeft.getPosition();
    this.m_leftVelocity = this.climberMotorLeft.getVelocity();
    this.m_rightPosition = this.climberMotorRight.getPosition();
    this.m_rightVelocity = this.climberMotorRight.getVelocity();

    ledTrigger = false;
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

  public void setMotorsPowers(double power) {
    this.climberMotorLeft.set(this.getLeftLimitSwitch()&&power<0?0:power);
    this.climberMotorRight.set(this.getRightLimitSwitch()&&power<0?0:power);
    SmartDashboard.putNumber("Elevator Power", power);
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putBoolean("Left Limit Switch", this.getLeftLimitSwitch());
    SmartDashboard.putBoolean("Right Limit Switch", this.getRightLimitSwitch());

  }
}
