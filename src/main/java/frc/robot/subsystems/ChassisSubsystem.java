// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ChassisSubsystem extends SubsystemBase {
  // An enum with the names of the wheel modules
  public enum wheels {
    left_front, right_front, right_back, left_back
  }
  // An array of the four swerve Modules
  private SwerveModule[] swerve_modules = new SwerveModule[4];

  // IMU
  private AHRS imu;
  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
          new SwerveModuleState(0,Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0,Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0,Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0,Rotation2d.fromDegrees(0))
  };
  public ChassisSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
