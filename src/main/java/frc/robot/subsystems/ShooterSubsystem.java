// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Util.LimelightUtil;


public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX shooterMotor; // Motor responsible for the intake and shooting of game pieces

  private StatusSignal<Double> m_position; // Status signal for shooter motor's position
  private StatusSignal<Double> m_velocity; // Status signal for shooter motor's velocity
  private LimelightUtil limelightShooter;

  public ShooterSubsystem() {
    this.shooterMotor = new TalonFX(ShooterConstants.kShooterMotorPort);

    this.m_position = this.shooterMotor.getPosition();
    this.m_velocity = this.shooterMotor.getVelocity();
    this.limelightShooter = new LimelightUtil("Shooting-Limelight");
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
     * Getter for the shooter's speed in rotations per minute
     * @return the RPM of the shooter
  */
  public double getShooterSpeedRPM() {
    this.m_velocity.refresh();

    return this.m_velocity.getValue()*60;
  }
  public LimelightUtil getLimelightShooter(){
    return limelightShooter;
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterRPM", getShooterSpeedRPM());
  }
}


