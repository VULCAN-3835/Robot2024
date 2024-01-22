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
  ShooterConstants Conn = new ShooterConstants();
  PIDController pid = new PIDController(Conn.kkPshooter,0 , 0);
  TalonFX shootermotor = new TalonFX(Conn.KMotorShooterPort);
  public double speedRPM;//A snitch who will always get the speed of the wheel
  /** Creates a new ShooterSubsystem. */
  public double error;

  public ShooterSubsystem() {}
  public void collect()
  {
    Conn.kDspeed=Conn.kCDspeedRPM;
  }
  public void stopMotor()
  {
    Conn.kDspeed =0.0;
  }
  public void setShooterSpeed(double speed){
    Conn.kDspeed =speed;
  }
  @Override
  public void periodic() {
    this.speedRPM = shootermotor.getVelocity().getValue()*600/Conn.KticksPerRotation;
    error = Conn.kDspeed -this.speedRPM;
    shootermotor.set(error*Conn.kkPshooter);
    // This method will be called once per scheduler run
  }
}
