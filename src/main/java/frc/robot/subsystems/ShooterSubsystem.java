// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX shooterMotor;
  private DoubleSolenoid ampPiston;

  public ShooterSubsystem() {
    this.shooterMotor = new TalonFX(Constants.ShooterConstants.kShooterMotorPort);
    this.ampPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH,Constants.ShooterConstants.kPistonForwardChannelNumber,Constants.ShooterConstants.kPistonReverseChannelNumber);
  }

  //Sets the motor to the speed of collect
  public void collect()
  {
    shooterMotor.set(Constants.ShooterConstants.kCollectPower);
  }
  //Stops the motor
  public void stopMotor(){
    shooterMotor.set(0);
  }

  /**
     * sets the motor to its desired speed
     * @param speed  the desired speed
  */
  public void setShooterSpeed(double shooterSpeed) { 
    this.shooterMotor.set(shooterSpeed);
  }
  /**
     * true opens the piston
     * false closes the piston
     * @param state the state of the piston
  */
  public void setPositionState(boolean state) {
    if (state) {
        ampPiston.set(DoubleSolenoid.Value.kForward); 
    } 
    else 
    {
        ampPiston.set(DoubleSolenoid.Value.kReverse); 
    }
  }
  @Override
  public void periodic() {
      
  }
}


