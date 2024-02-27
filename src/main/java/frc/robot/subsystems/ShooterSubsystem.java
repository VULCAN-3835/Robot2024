// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX shooterMotor;
  private DoubleSolenoid ampPiston;

  private StatusSignal<Double> m_position;
  private StatusSignal<Double> m_velocity;

  public ShooterSubsystem() {
    this.shooterMotor = new TalonFX(ShooterConstants.kShooterMotorPort);

    this.ampPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH,ShooterConstants.kPistonForwardChannelNumber,ShooterConstants.kPistonReverseChannelNumber);

    this.m_position = this.shooterMotor.getPosition();
    this.m_velocity = this.shooterMotor.getVelocity();
  }

  /**
     * sets the motor to the speed of collect
  */
  public void collect()
  {
    shooterMotor.set(ShooterConstants.kCollectPower);
  }
  /**
     * Stops the motor
  */
  public void stopMotor(){
    shooterMotor.set(0);
  }

  /**
     * sets the motor to its desired speed
     * @param speed  the desired speed
  */
  public void setShooterSpeed(double speed) { 
    shooterMotor.set(speed);
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
  public double getShooterSpeedRPM() {
    this.m_velocity.refresh();

    return this.m_velocity.getValue()*60;
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterRPM", getShooterSpeedRPM());
  }
}


