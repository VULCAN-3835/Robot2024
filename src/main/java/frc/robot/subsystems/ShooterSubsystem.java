// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  //The calculation is without a conversion ratio because Model told me it's 1:1
  //The radius of the shooting wheels is 2 inches x 5.08 cm The circumference of the wheels of the shooting is 31.918581324 cm
  private PIDController pid;//pidcontroler
  private TalonFX shootermotor; //set motor
  private double speedRPM;//A snitch who will always get the speed of the wheel
  private DoubleSolenoid shooterPiston;//set piston

  public ShooterSubsystem() {
    this.pid = new PIDController(ShooterConstants.kPshooter,0 , 0);
    this.shootermotor = new TalonFX(ShooterConstants.KMotorShooterPort);
    pid.setSetpoint(0.0);
    this.shooterPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,ShooterConstants.KDoubleSelenoidForward,ShooterConstants.KDoubleSelenoidReverse);
  }

  public void setPositionState(boolean state)// true- open the piston,false- close the piston
  {
    if(state)
    {
      shooterPiston.set(DoubleSolenoid.Value.kForward);
    }else
    {
      shooterPiston.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void collect()//set the speet to collect,has constants so it can be changed
  {
    pid.setSetpoint(ShooterConstants.kCollectSpdRPM);

  }

  public void stopMotor()//stop thee motor movemnt
  {
    pid.setSetpoint(0.0);
  }

  public void setShooterSpeed(double speed){//set the motor to its desierd speed, speed can be changed  in constants
    pid.setSetpoint(speed);
  }

  @Override
  public void periodic() {
    this.speedRPM = shootermotor.getVelocity().getValue()*600/ShooterConstants.KticksPerRotation;//Calculates the speed of the motor
    shootermotor.set(pid.calculate(this.speedRPM));
    // This method will be called once per scheduler run
  }
}
