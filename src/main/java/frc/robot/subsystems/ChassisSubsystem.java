// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Util.SwerveModule;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class ChassisSubsystem extends SubsystemBase {
  // An enum with the names of the wheel modules
  public enum Wheels {
    LEFT_FRONT, RIGHT_FRONT, RIGHT_BACK, LEFT_BACK
  }

  // An array of the four swerve Modules
  private SwerveModule[] swerve_modules = new SwerveModule[4];

  // An array of the four swerve module's positions
  private SwerveModulePosition[] swerve_positions = new SwerveModulePosition[4];

  // Inertial Measurement unit 
  private AHRS imu;

  // Pose estimator responsible for keeping the robot's position on the field using gyro, encoders and camera detection
  private SwerveDrivePoseEstimator poseEstimator;

  private Field2d field;

  // The states of the modules
  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
          new SwerveModuleState(0,Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0,Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0,Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0,Rotation2d.fromDegrees(0))
  };

  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  SysIdRoutine routine;
  public ChassisSubsystem() {
    // Modules Initilization:
    this.swerve_modules[Wheels.LEFT_FRONT.ordinal()] = new SwerveModule(
      Constants.ChassisConstants.kLeftFrontDriveID,
      Constants.ChassisConstants.kLeftFrontSteerID, 
      Constants.ChassisConstants.kLeftFrontEncID,
      Constants.ChassisConstants.kLeftFrontInverted, 
      Constants.ChassisConstants.kLeftFrontOffset);

    this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()] = new SwerveModule(
      Constants.ChassisConstants.kRightFrontDriveID,
      Constants.ChassisConstants.kRightFrontSteerID, 
      Constants.ChassisConstants.kRightFrontEncID,
      Constants.ChassisConstants.kRightFrontInverted, 
      Constants.ChassisConstants.kRightFrontOffset);

    this.swerve_modules[Wheels.LEFT_BACK.ordinal()] = new SwerveModule(
      Constants.ChassisConstants.kLeftBackDriveID, 
      Constants.ChassisConstants.kLeftBackSteerID,
      Constants.ChassisConstants.kLeftBackEncID,
      Constants.ChassisConstants.kLeftBackInverted,
      Constants.ChassisConstants.kLeftBackOffset);

    this.swerve_modules[Wheels.RIGHT_BACK.ordinal()] = new SwerveModule(
      Constants.ChassisConstants.kRightBackDriveID,
      Constants.ChassisConstants.kRightBackSteerID,
      Constants.ChassisConstants.kRightBackEncID,
      Constants.ChassisConstants.kRightBackInverted,
      Constants.ChassisConstants.kRightBackOffset);

    // Imu initlization
    this.imu = new AHRS();
    field = new Field2d();

    updateSwervePositions();
    this.poseEstimator = new SwerveDrivePoseEstimator(ChassisConstants.kDriveKinematics,
      getRotation2d().unaryMinus(),
      this.swerve_positions,
      new Pose2d());

    zeroHeading();
    
    routine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(this::setSysidVolt, 
    log -> {
      // set states for all 4 modules
      for (Wheels wheel : Wheels.values()) {
      log.motor(wheel.toString())
          .voltage(
              m_appliedVoltage.mut_replace(
                  swerve_modules[wheel.ordinal()].getVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(swerve_modules[wheel.ordinal()].getPosition().distanceMeters, Meters))
          .linearVelocity(
              m_velocity.mut_replace(swerve_modules[wheel.ordinal()].getVelocity(), MetersPerSecond)); } }
              , this));
}

  private void updateSwervePositions() {
    this.swerve_positions[Wheels.LEFT_FRONT.ordinal()] = this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].getPosition();
    this.swerve_positions[Wheels.RIGHT_FRONT.ordinal()] = this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getPosition();
    this.swerve_positions[Wheels.LEFT_BACK.ordinal()] = this.swerve_modules[Wheels.LEFT_BACK.ordinal()].getPosition();
    this.swerve_positions[Wheels.RIGHT_BACK.ordinal()] = this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].getPosition();

  }

  /**
     * Resets the heading of the robot
  */
  public void zeroHeading() {
    this.imu.reset();
  }

  /**
     * Returns the heading of the robot
     * @return The heading of the robot in degrees
  */
  public double getHeading() {
    return this.imu.getAngle();
  }

  /**
     * Returns the heading of the robot
     * @return The heading of the robot in Rotation2d
  */
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
            fieldRelative? ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rot, this.imu.getRotation2d())
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
          fieldRelative?ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, this.imu.getRotation2d())
          :chassisSpeeds
      );
    }

    /**
     * Sets the desired states of the modules to given ones
     *
     * @param desiredStates  Desired array of 4 module states
     */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Sets max acceleration and velocity to Wheels
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.ChassisConstants.kMaxDrivingVelocity);
    
    // Uses the set method of the SwerveModule to declare desired state of the module
    this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].set(desiredStates[0]);
    this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].set(desiredStates[1]);
    this.swerve_modules[Wheels.LEFT_BACK.ordinal()].set(desiredStates[2]);
    this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].set(desiredStates[3]);
  }

  private void setSysidVolt(Measure<Voltage> volts) {
      double voltageDouble = volts.magnitude();
      setModulesVoltage(voltageDouble);
  }
  
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return this.routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return this.routine.dynamic(direction);
  }

  public void setModulesVoltage(double volt) {
    this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].setMotorVoltage(volt);
    this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].setMotorVoltage(volt);
    this.swerve_modules[Wheels.LEFT_BACK.ordinal()].setMotorVoltage(volt);
    this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].setMotorVoltage(volt);
  }

  @Override
  public void periodic() {
    setModuleStates(this.swerveModuleStates);

    updateSwervePositions();
    this.poseEstimator.update(getRotation2d().unaryMinus(), this.swerve_positions);
    
    this.field.setRobotPose(this.poseEstimator.getEstimatedPosition());
    SmartDashboard.putData(field);

    SmartDashboard.putNumber("Left Front Distance",this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].getPosition().distanceMeters);
    SmartDashboard.putNumber("Left Back Distance",this.swerve_modules[Wheels.LEFT_BACK.ordinal()].getPosition().distanceMeters);
    SmartDashboard.putNumber("Right Front Distance",this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getPosition().distanceMeters);
    SmartDashboard.putNumber("Right Back Distance",this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].getPosition().distanceMeters);

    SmartDashboard.putNumber("Left Front Rotation",this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].getPosition().angle.getRotations());
    SmartDashboard.putNumber("Left Back Rotation",this.swerve_modules[Wheels.LEFT_BACK.ordinal()].getPosition().angle.getRotations());
    SmartDashboard.putNumber("Right Front Rotation",this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getPosition().angle.getRotations());
    SmartDashboard.putNumber("Right Back Rotation",this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].getPosition().angle.getRotations());

    SmartDashboard.putNumber("Left Front Error",this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].getModuleAngleError());
    SmartDashboard.putNumber("Left Back Error",this.swerve_modules[Wheels.LEFT_BACK.ordinal()].getModuleAngleError());
    SmartDashboard.putNumber("Right Front Error",this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getModuleAngleError());
    SmartDashboard.putNumber("Right Back Error",this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].getModuleAngleError());

    SmartDashboard.putNumber("Left Front Output",this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].getModuleClosedLoopOutput());
    SmartDashboard.putNumber("Left Back Output",this.swerve_modules[Wheels.LEFT_BACK.ordinal()].getModuleClosedLoopOutput());
    SmartDashboard.putNumber("Right Front Output",this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getModuleClosedLoopOutput());
    SmartDashboard.putNumber("Right Back Output",this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].getModuleClosedLoopOutput());
  
    SmartDashboard.putNumber("Left Front Voltage", swerve_modules[Wheels.LEFT_FRONT.ordinal()].getVoltage());
    SmartDashboard.putNumber("Left Front Distance", swerve_modules[Wheels.LEFT_FRONT.ordinal()].getPosition().distanceMeters);
    SmartDashboard.putNumber("Left Front Velocity", swerve_modules[Wheels.LEFT_FRONT.ordinal()].getVelocity());
  }
}
