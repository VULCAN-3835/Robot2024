// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private TalonFX climberMotorRight;  // Motor for the right side of the climber
  private TalonFX climberMotorLeft;   // Motor for the left side of the climber
  private DigitalInput limitSwitchRight; // Limit switch for the right side
  private DigitalInput limitSwitchLeft;  // Limit switch for the left side

  private boolean ledTrigger;  // Flag to control LED behavior
  
  private StatusSignal<Double> m_leftPosition;  // Status signal for left motor position
  private StatusSignal<Double> m_leftVelocity;  // Status signal for left motor velocity
  private StatusSignal<Double> m_rightPosition; // Status signal for right motor position
  private StatusSignal<Double> m_rightVelocity; // Status signal for right motor velocity

  MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);  // Motion Magic voltage control configuration

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    // Initialize TalonFX motors for both sides of the climber
    this.climberMotorRight = new TalonFX(Constants.ClimberConstants.kRightMotorPort);
    this.climberMotorLeft = new TalonFX(Constants.ClimberConstants.kLeftMotorPort);

    // Initialize limit switches for both sides
    this.limitSwitchRight = new DigitalInput(Constants.ClimberConstants.kRightSwitchPort);
    this.limitSwitchLeft = new DigitalInput(Constants.ClimberConstants.kLeftSwitchPort);
      
    // Configure TalonFX motor settings
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    
    // Set the feedback sensor to return rotations of the axis
    configuration.Feedback.SensorToMechanismRatio = Constants.ClimberConstants.kMotorRatio;

    // Set Motion Magic parameters
    configuration.Slot0 = Constants.ClimberConstants.getElevatorSlot();
    configuration.MotionMagic.MotionMagicAcceleration = Constants.ClimberConstants.kElevatorAcceleration;
    configuration.MotionMagic.MotionMagicCruiseVelocity = Constants.ClimberConstants.kElevatorMaxCruiseVelocity;

    // Set motor output neutral mode to Brake
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    // Apply configuration to left and right climber motors
    this.climberMotorLeft.getConfigurator().apply(configuration);
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    this.climberMotorRight.getConfigurator().apply(configuration);

    // Initialize status signals for motor position and velocity
    this.m_leftPosition = this.climberMotorLeft.getPosition();
    this.m_leftVelocity = this.climberMotorLeft.getVelocity();
    this.m_rightPosition = this.climberMotorRight.getPosition();
    this.m_rightVelocity = this.climberMotorRight.getVelocity();

    // Initialize LED trigger flag
    ledTrigger = false;
  }

  /** 
   * Getter for the right side limit switch of the climber
   * @return True if pressed
  */
  public boolean getRightLimitSwitch(){
    return this.limitSwitchRight.get();
  }

  /** 
   * Getter for the left side limit switch of the climber
   * @return True if pressed
  */
  public boolean getLeftLimitSwitch(){
    return this.limitSwitchLeft.get();
  }

  /** 
   * Getter for the current position of the left climber motor in rotations
   * @return The current position of the left climber motor in rotations
  */
  public double getLeftRotations() {
    this.m_leftPosition.refresh();  // Refresh position data
    this.m_leftVelocity.refresh();  // Refresh velocity data

    double pos = StatusSignal.getLatencyCompensatedValue(m_leftPosition, m_leftVelocity);  // Get compensated position value

    return pos;
  }

  /** 
   * Getter for the current position of the right climber motor in rotations
   * @return The current position of the right climber motor in rotations
  */
  public double getRightRotations() {
    this.m_rightPosition.refresh();  // Refresh position data
    this.m_rightVelocity.refresh();  // Refresh velocity data

    double pos = StatusSignal.getLatencyCompensatedValue(m_rightPosition, m_rightVelocity);  // Get compensated position value

    return pos;
  }

  /** 
   * Setter for the target position for both climber motors
   * @param pos The target position of the climbers
  */
  public void setClimberPosition(double pos){
    this.climberMotorLeft.setControl(this.motionMagicVoltage.withPosition(pos));  // Set position control for left motor
    this.climberMotorRight.setControl(this.motionMagicVoltage.withPosition(pos));  // Set position control for right motor
  }

  /** 
   * Setter for the power to give the climber motors considering limit switch states
   * @param power The power in precentage to apply to motors
  */
  public void setMotorsPowers(double power) {
    // Stop motor if limit switch is pressed and power is negative
    this.climberMotorLeft.set(this.getLeftLimitSwitch() && power < 0 ? 0 : power);
    this.climberMotorRight.set(this.getRightLimitSwitch() && power < 0 ? 0 : power);
    SmartDashboard.putNumber("Elevator Power", power);  // Display power on SmartDashboard
  }

  @Override
  public void periodic() {
    // Update SmartDashboard with the state of limit switches
    SmartDashboard.putBoolean("Left Limit Switch", this.getLeftLimitSwitch());
    SmartDashboard.putBoolean("Right Limit Switch", this.getRightLimitSwitch());
  }
}
