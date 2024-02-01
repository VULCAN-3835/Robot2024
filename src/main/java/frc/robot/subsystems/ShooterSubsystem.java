// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  //The calculation is without a conversion ratio because Model told me it's 1:1
  //The radius of the shooting wheels is 2 inches x 5.08 cm The circumference of the wheels of the shooting is 31.918581324 cm
  private TalonFX shooterMotor;
  private DoubleSolenoid piston;
  private PIDController velocityController;

  public ShooterSubsystem() {
    this.shooterMotor = new TalonFX(Constants.ShooterSubsystemConstants.kShooterMotorPort);
    this.shooterMotor.setInverted(false); //Depending on where the motor is placed Invert or delete this line

    this.piston = new DoubleSolenoid(PneumaticsModuleType.REVPH,Constants.ShooterSubsystemConstants.kPistonForwardChannelNumber,Constants.ShooterSubsystemConstants.kPistonReverseChannelNumber);
    this.velocityController = new PIDController(Constants.ShooterSubsystemConstants.kVelocityPIDKp, 0, 0);
  }
  public void collect()//Set the motor speed to set constant
  {
    velocityController.setSetpoint(Constants.ShooterSubsystemConstants.kCollectSpdRPM);
  }
  public void stopMotor()//Stop the motor
  {
    velocityController.setSetpoint(0.0);
  }
  public void setShooterSpeed(double speed) { //set the motor to a desired speed
    velocityController.setSetpoint(speed);
  }
  public void setPositionState(boolean state) {
    if (state) {
        piston.set(DoubleSolenoid.Value.kForward); //Opens the piston
    } 
    else 
    {
        piston.set(DoubleSolenoid.Value.kReverse); //Closes the piston
    }
  }
  @Override
  public void periodic() {
    shooterMotor.set(velocityController.calculate(shooterMotor.getVelocity().getValue() * 600 / Constants.ShooterSubsystemConstants.kTicksPerRotation));
    //The value inside of velocityController.calculate is the motor speed
  }
}