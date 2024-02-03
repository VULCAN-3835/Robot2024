// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterSubsystemConstants;;

public class ShooterSubsystem extends SubsystemBase {
  //The calculation is without a conversion ratio because Model told me it's 1:1
  //The radius of the shooting wheels is 2 inches x 5.08 cm The circumference of the wheels of the shooting is 31.918581324 cm
  private TalonFX shooterMotorLeft;
  private TalonFX shooterMotorRight;
  private DoubleSolenoid ampPiston;
  private PIDController velocityController;

  public ShooterSubsystem() {
    this.shooterMotorLeft = new TalonFX(ShooterSubsystemConstants.kShooterMotorPortLeft);
    this.shooterMotorLeft.setInverted(false); //Depending on where the motor is placed Invert or delete this line
    this.shooterMotorRight = new TalonFX(ShooterSubsystemConstants.kShooterMotorPortright);
    
    this.ampPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH,ShooterSubsystemConstants.kPistonForwardChannelNumber,ShooterSubsystemConstants.kPistonReverseChannelNumber);
    this.velocityController = new PIDController(ShooterSubsystemConstants.kVelocityPIDKp, 0, 0);
  }

  public void collect()//Set the motor speed to set constant
  {
    shooterMotorLeft.set(ShooterSubsystemConstants.kCollectSpdRPM);
    shooterMotorRight.set(-ShooterSubsystemConstants.kCollectSpdRPM);
  }

  public void stopMotor(){//Stop the motor
    shooterMotorLeft.set(0);
    shooterMotorRight.set(0);
  }

  public void setShooterSpeed(double speed) { //set the motor to a desired speed
    shooterMotorLeft.set(speed);
    shooterMotorRight.set(-speed);
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
    shooterMotorLeft.set(
      velocityController.calculate(shooterMotorLeft.getVelocity().getValue() * 600 / ShooterSubsystemConstants.kTicksPerRotation));
    //The value inside of velocityController.calculate is the motor speed
  }
}