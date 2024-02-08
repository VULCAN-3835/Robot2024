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
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  //The calculation is without a conversion ratio because Model told me it's 1:1
  //The radius of the shooting wheels is 2 inches x 5.08 cm The circumference of the wheels of the shooting is 31.918581324 cm
  private final TalonFX shooterMotor;
  private DoubleSolenoid ampPiston;

  public ShooterSubsystem() {
    this.shooterMotor = new TalonFX(ShooterConstants.kShooterMotorPort);
    this.ampPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH,ShooterConstants.kPistonForwardChannelNumber,ShooterConstants.kPistonReverseChannelNumber);
  }

  /**
     * sets the motor to the speed of collect
  */
  public void collect()
  {
    shooterMotor.set(ShooterConstants.kCollectSpd);
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
  @Override
  public void periodic() {

  }
}


