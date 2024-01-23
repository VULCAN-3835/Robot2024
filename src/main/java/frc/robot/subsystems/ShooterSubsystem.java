// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  //The calculation is without a conversion ratio because Model told me it's 1:1
  //The radius of the shooting wheels is 2 inches x 5.08 cm The circumference of the wheels of the shooting is 31.918581324 cm
  private PIDController pid;
  private TalonFX shootermotor; 
  private double speedRPM;//A snitch who will always get the speed of the wheel
  public ShooterSubsystem() {
    this.pid = new PIDController(ShooterConstants.kPshooter,0 , 0);
    this.shootermotor = new TalonFX(ShooterConstants.KMotorShooterPort);
    pid.setSetpoint(0.0);
  }
  public void collect()
  {
    pid.setSetpoint(ShooterConstants.kCollectSpdRPM);

  }
  public void stopMotor()
  {
    pid.setSetpoint(0.0);
  }
  public void setShooterSpeed(double speed){
    pid.setSetpoint(speed);
  }
  @Override
  public void periodic() {
    this.speedRPM = shootermotor.getVelocity().getValue()*600/ShooterConstants.KticksPerRotation;
    shootermotor.set(pid.calculate(this.speedRPM));
    // This method will be called once per scheduler run
  }
}
