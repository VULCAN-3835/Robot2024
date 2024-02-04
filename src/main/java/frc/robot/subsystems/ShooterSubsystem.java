// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  //The calculation is without a conversion ratio because Model told me it's 1:1
  //The radius of the shooting wheels is 2 inches x 5.08 cm The circumference of the wheels of the shooting is 31.918581324 cm
  private TalonFX shooterMotorLeft;
  private TalonFX shooterMotorRight;
  private DoubleSolenoid ampPiston;

  public ShooterSubsystem() {
    this.shooterMotorLeft = new TalonFX(ShooterConstants.kShooterMotorPortLeft);
    this.shooterMotorLeft.setInverted(true); 
    this.shooterMotorLeft.setNeutralMode(NeutralModeValue.Brake);

    this.shooterMotorRight = new TalonFX(ShooterConstants.kShooterMotorPortRight);
    this.shooterMotorRight.setInverted(false);
    this.shooterMotorRight.setNeutralMode(NeutralModeValue.Brake);
    
    this.ampPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH,ShooterConstants.kPistonForwardChannelNumber,ShooterConstants.kPistonReverseChannelNumber);
  }

  public void collect() //Set the motor speed to set constant
  {
    shooterMotorLeft.set(ShooterConstants.kCollectSpd);
    shooterMotorRight.set(ShooterConstants.kCollectSpd);
  }

  public void stopMotor(){ //Stop the motor
    shooterMotorLeft.set(0);
    shooterMotorRight.set(0);
  }

  public void setShooterSpeed(double speed) { //set the motor to a desired speed
    shooterMotorLeft.set(speed);
    shooterMotorRight.set(speed);
  }

  public void setPositionState(boolean state) {
    if (state) {
        ampPiston.set(DoubleSolenoid.Value.kForward); //Opens the piston
    } 
    else 
    {
        ampPiston.set(DoubleSolenoid.Value.kReverse); //Closes the piston
    }
  }
  @Override
  public void periodic() {

  }
}