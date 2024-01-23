// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.SwerveModule;

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
    this.swerve_modules[wheels.left_front.ordinal()] = new SwerveModule(
      Constants.ChassisConstants.kLeftFrontDriveID,
      Constants.ChassisConstants.kLeftFrontSteerID, 
      Constants.ChassisConstants.kLeftFrontEncID,
      Constants.ChassisConstants.kLeftFrontInverted, 
      Constants.ChassisConstants.kLeftFrontOffset);

    this.swerve_modules[wheels.right_front.ordinal()] = new SwerveModule(
      Constants.ChassisConstants.kRightFrontDriveID,
      Constants.ChassisConstants.kRightFrontSteerID, 
      Constants.ChassisConstants.kRightFrontEncID,
      Constants.ChassisConstants.kRightFrontInverted, 
      Constants.ChassisConstants.kRightFrontOffset);

    this.swerve_modules[wheels.left_back.ordinal()] = new SwerveModule(
      Constants.ChassisConstants.kLeftBackDriveID, 
      Constants.ChassisConstants.kLeftBackSteerID,
      Constants.ChassisConstants.kLeftBackEncID,
      Constants.ChassisConstants.kLeftBackInverted,
      Constants.ChassisConstants.kLeftBackOffset);

    this.swerve_modules[wheels.right_back.ordinal()] = new SwerveModule(
      Constants.ChassisConstants.kRightBackDriveID,
      Constants.ChassisConstants.kRightBackSteerID,
      Constants.ChassisConstants.kRightBackEncID,
      Constants.ChassisConstants.kRightBackInverted,
      Constants.ChassisConstants.kRightBackOffset);
  }

  public void zeroHeading() {
    this.imu.reset();
  }

  public double getHeading() {
    return this.imu.getAngle();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(this.imu.getAngle());
  }

  /**
     * Sets the module's state to given one
     *
     * @param xVelocity  The velocity on the x axis
     * @param yVelocity  The velocity on the y axis
     * @param rot  The rotational velocity
     * @param fieldRelative  Is field relative or not
     */
  public void drive(double xVelocity, double yVelocity, double rot, boolean fieldRelative) {
    // Kinematics turns the Chassis speeds to desired swerveModule states depending on if field relative or not
    this.swerveModuleStates =
    Constants.ChassisConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative? ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rot, getRotation2d())
                : new ChassisSpeeds(xVelocity, yVelocity, rot));

  }

  /**
     * Sets the module's state to given one
     *
     * @param chassisSpeeds The desired chassisSpeeds object for module velocities
     * @param fieldRelative  Is field relative or not
     */
    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
      // Makes a swerve module-state array from chassisSpeeds
        this.swerveModuleStates = 
        Constants.ChassisConstants.kDriveKinematics.toSwerveModuleStates(
          fieldRelative?ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getRotation2d())
          :chassisSpeeds
      );
    }

    /**
     * Sets the desired states of the modules to given ones
     *
     * @param desiredStates  Desired array of 4 module states
     */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Sets max acceleration and velocity to wheels
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.ChassisConstants.kMaxDrivingVelocity);
    
    // Uses the set method of the SwerveModule to declare desired state of the module
    this.swerve_modules[wheels.left_front.ordinal()].set(desiredStates[0]);
    this.swerve_modules[wheels.right_front.ordinal()].set(desiredStates[1]);
    this.swerve_modules[wheels.left_back.ordinal()].set(desiredStates[2]);
    this.swerve_modules[wheels.right_back.ordinal()].set(desiredStates[3]);
  }

  @Override
  public void periodic() {
    setModuleStates(this.swerveModuleStates);


    SmartDashboard.putNumber("Left Front Rotation",this.swerve_modules[wheels.left_front.ordinal()].getModuleAngle());
    SmartDashboard.putNumber("Left Back Rotation",this.swerve_modules[wheels.left_back.ordinal()].getModuleAngle());
    SmartDashboard.putNumber("Right Front Rotation",this.swerve_modules[wheels.right_front.ordinal()].getModuleAngle());
    SmartDashboard.putNumber("Right Back Rotation",this.swerve_modules[wheels.right_back.ordinal()].getModuleAngle());

    SmartDashboard.putNumber("Left Front Error",this.swerve_modules[wheels.left_front.ordinal()].getModuleAngleError());
    SmartDashboard.putNumber("Left Back Error",this.swerve_modules[wheels.left_back.ordinal()].getModuleAngleError());
    SmartDashboard.putNumber("Right Front Error",this.swerve_modules[wheels.right_front.ordinal()].getModuleAngleError());
    SmartDashboard.putNumber("Right Back Error",this.swerve_modules[wheels.right_back.ordinal()].getModuleAngleError());

    SmartDashboard.putNumber("Left Front Output",this.swerve_modules[wheels.left_front.ordinal()].getModuleClosedLoopOutput());
    SmartDashboard.putNumber("Left Back Output",this.swerve_modules[wheels.left_back.ordinal()].getModuleClosedLoopOutput());
    SmartDashboard.putNumber("Right Front Output",this.swerve_modules[wheels.right_front.ordinal()].getModuleClosedLoopOutput());
    SmartDashboard.putNumber("Right Back Output",this.swerve_modules[wheels.right_back.ordinal()].getModuleClosedLoopOutput());
  }
}
