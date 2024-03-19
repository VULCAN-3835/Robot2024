// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Util.LimelightUtil;
import frc.robot.Util.SwerveModule;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

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

  private Pose2d startingPos;

  // Field object for presenting position relative to field
  private Field2d field;

  private boolean hasNotReset = true;

  // The states of the modules
  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
          new SwerveModuleState(0,Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0,Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0,Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0,Rotation2d.fromDegrees(0))
  };

  private LimelightUtil limelight;
  private Field2d llField;

  // Sysid Measurements
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  // Sysid Rotinue
  SysIdRoutine routine;

  public ChassisSubsystem() {
    // Modules Initilization:
    this.swerve_modules[Wheels.LEFT_FRONT.ordinal()] = new SwerveModule(
      Constants.ChassisConstants.kLeftFrontDriveID,
      Constants.ChassisConstants.kLeftFrontSteerID, 
      Constants.ChassisConstants.kLeftFrontEncID,
      Constants.ChassisConstants.kLeftFrontInverted, 
      Constants.ChassisConstants.kLeftFrontOffset,
      ModuleConstants.leftFrontFF);

    this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()] = new SwerveModule(
      Constants.ChassisConstants.kRightFrontDriveID,
      Constants.ChassisConstants.kRightFrontSteerID, 
      Constants.ChassisConstants.kRightFrontEncID,
      Constants.ChassisConstants.kRightFrontInverted, 
      Constants.ChassisConstants.kRightFrontOffset,
      ModuleConstants.rightFrontFF);

    this.swerve_modules[Wheels.LEFT_BACK.ordinal()] = new SwerveModule(
      Constants.ChassisConstants.kLeftBackDriveID, 
      Constants.ChassisConstants.kLeftBackSteerID,
      Constants.ChassisConstants.kLeftBackEncID,
      Constants.ChassisConstants.kLeftBackInverted,
      Constants.ChassisConstants.kLeftBackOffset,
      ModuleConstants.leftBackFF);

    this.swerve_modules[Wheels.RIGHT_BACK.ordinal()] = new SwerveModule(
      Constants.ChassisConstants.kRightBackDriveID,
      Constants.ChassisConstants.kRightBackSteerID,
      Constants.ChassisConstants.kRightBackEncID,
      Constants.ChassisConstants.kRightBackInverted,
      Constants.ChassisConstants.kRightBackOffset,
      ModuleConstants.rightBackFF);

    // Imu initlization
    this.imu = new AHRS();
    
    // Field initlization
    field = new Field2d();

    // Update swerve position and heading at build
    updateSwervePositions();
    zeroHeading();

    limelight = new LimelightUtil("limelight-front");

    // Robot starting position for odometry
    startingPos = new Pose2d(1.3, 5.52, Rotation2d.fromDegrees(0));
    
    // Initilizing a pose estimator
    this.poseEstimator = new SwerveDrivePoseEstimator(ChassisConstants.kDriveKinematics,
      getRotation2d().unaryMinus(),
      this.swerve_positions,
      startingPos);

    // Configuring the controller for the path planner
    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetOdometry, 
      () -> ChassisConstants.kDriveKinematics.toChassisSpeeds(getModStates()), 
      this::runVelc,
      new HolonomicPathFollowerConfig(
        new PIDConstants(1.75, 0, 0), // Translation PID
        new PIDConstants(5.0, 0, 0), // Rotation PID
        4.0, 
        ChassisConstants.kWheelRadius, 
        new ReplanningConfig()),
      () -> !(Robot.allianceColor == "BLUE"), 
      this);

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    SmartDashboard.putData(field);

    llField = new Field2d();
    SmartDashboard.putData("Limelight Field", llField);

    
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
    var invert = 1;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red && fieldRelative)
      invert = -1;
    // Kinematics turns the Chassis speeds to desired swerveModule states depending on if field relative or not
    this.swerveModuleStates =
    Constants.ChassisConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative? ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity*invert, yVelocity*invert, rot, this.imu.getRotation2d())
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
     * Runs the robot following trajectory
     *
     * @param chassisSpeeds The desired chassisSpeeds object for module velocities
  */
    public void runVelc(ChassisSpeeds speeds) {
      ChassisSpeeds discSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

      this.swerveModuleStates = ChassisConstants.kDriveKinematics.toSwerveModuleStates(discSpeeds);
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
  
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return this.routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return this.routine.dynamic(direction);
  }

  private void setSysidVolt(Measure<Voltage> volts) {
    double voltageDouble = volts.magnitude();
    setModulesVoltage(voltageDouble);
  }

  public void setModulesVoltage(double volt) {
    this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].setMotorVoltage(volt);
    this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].setMotorVoltage(volt);
    this.swerve_modules[Wheels.LEFT_BACK.ordinal()].setMotorVoltage(volt);
    this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].setMotorVoltage(volt);
  }

  public LimelightUtil getLimelight() {
    return this.limelight;
  }

  private void resetOdometry(Pose2d pose) {
    this.poseEstimator.resetPosition(getRotation2d().unaryMinus(), getModPositions(), pose);
  }

  public Pose2d getPose() {
    return this.poseEstimator.getEstimatedPosition();
  }

  public SwerveModulePosition[] getModPositions() {
    return this.swerve_positions;
  }

  public SwerveModuleState[] getModStates() {
    return this.swerveModuleStates;
  }

  private void updatePoseEstimatorWithVisionBotPose() {
    Pose2d visionBotPose = this.limelight.getPoseFromCamera();
    if (visionBotPose.getX() == 0.0) {
      return;
    }

    // distance from current pose to vision estimated pose
    double poseDifference = poseEstimator.getEstimatedPosition().getTranslation()
        .getDistance(visionBotPose.getTranslation());

    if (this.limelight.cameraHasTarget()) {
      double xyStds;
      double degStds;

      xyStds = 0.5;
      degStds = 6;
      // if (false) {
      //   xyStds = 0.5;
      //   degStds = 6;
      // }
      // // 1 target with large area and close to estimated pose
      // else if (this.limelight.getA() > 0.8 && poseDifference < 0.5) {
      //   xyStds = 0.75;
      //   degStds = 10;
      // }
      // // 1 target farther away and estimated pose is close
      // else if (this.limelight.getA() > 0.1 && poseDifference < 0.3) {
      //   xyStds = 1.0;
      //   degStds = 20;
      // }
      // conditions don't match to add a vision measurement
      // else {
      //   return;
      // }

      poseEstimator.setVisionMeasurementStdDevs(
          VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
      poseEstimator.addVisionMeasurement(visionBotPose,
          Timer.getFPGATimestamp() - (this.limelight.getCameraTimeStampSec()));
    }
  }

  @Override
  public void periodic() {
    setModuleStates(this.swerveModuleStates);
    
    updateSwervePositions();
    this.poseEstimator.update(getRotation2d().unaryMinus(), this.swerve_positions);
    updatePoseEstimatorWithVisionBotPose();

    // if (this.limelight.hasValidTarget()) {
    //   this.poseEstimator.addVisionMeasurement(this.limelight.getPoseFromCamera(), Timer.getFPGATimestamp() - (this.limelight.getCameraTimeStampSec()));
    // }
    this.field.setRobotPose(this.poseEstimator.getEstimatedPosition());
    this.llField.setRobotPose(this.limelight.getPoseFromCamera());

    Logger.recordOutput("Swerve/SwerveStates", swerveModuleStates);
    Logger.recordOutput("Swerve/Heading", imu.getAngle());

    Logger.recordOutput("Field/RobotPose", getPose());
    Logger.recordOutput("Field/CameraA", limelight.getA());
    
    SmartDashboard.putNumber("Gyro Heading", getHeading());

    SmartDashboard.putNumber("Left Front Distance",this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].getPosition().distanceMeters);
    SmartDashboard.putNumber("Left Back Distance",this.swerve_modules[Wheels.LEFT_BACK.ordinal()].getPosition().distanceMeters);
    SmartDashboard.putNumber("Right Front Distance",this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getPosition().distanceMeters);
    SmartDashboard.putNumber("Right Back Distance",this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].getPosition().distanceMeters);

    SmartDashboard.putNumber("Left Front Rotation",this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].getPosition().angle.getRotations());
    SmartDashboard.putNumber("Left Back Rotation",this.swerve_modules[Wheels.LEFT_BACK.ordinal()].getPosition().angle.getRotations());
    SmartDashboard.putNumber("Right Front Rotation",this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getPosition().angle.getRotations());
    SmartDashboard.putNumber("Right Back Rotation",this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].getPosition().angle.getRotations());

    SmartDashboard.putNumber("Left Front Rotation Error",this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].getModuleAngleError());
    SmartDashboard.putNumber("Left Back Rotation Error",this.swerve_modules[Wheels.LEFT_BACK.ordinal()].getModuleAngleError());
    SmartDashboard.putNumber("Right Front Rotation Error",this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getModuleAngleError());
    SmartDashboard.putNumber("Right Back Rotation Error",this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].getModuleAngleError());

    SmartDashboard.putNumber("Left Front Rotation Output",this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].getModuleClosedLoopOutput());
    SmartDashboard.putNumber("Left Back Rotation Output",this.swerve_modules[Wheels.LEFT_BACK.ordinal()].getModuleClosedLoopOutput());
    SmartDashboard.putNumber("Right Front Rotation Output",this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getModuleClosedLoopOutput());
    SmartDashboard.putNumber("Right Back Rotation Output",this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].getModuleClosedLoopOutput());
  
    SmartDashboard.putNumber("Left Front Drive Velocity", swerve_modules[Wheels.LEFT_FRONT.ordinal()].getVelocity());
    SmartDashboard.putNumber("Left Back Drive Velocity", swerve_modules[Wheels.LEFT_BACK.ordinal()].getVelocity());
    SmartDashboard.putNumber("Right Front Drive Velocity", swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getVelocity());
    SmartDashboard.putNumber("Right Back Drive Velocity", swerve_modules[Wheels.RIGHT_BACK.ordinal()].getVelocity());

    SmartDashboard.putNumber("Left Front Drive Output", swerve_modules[Wheels.LEFT_FRONT.ordinal()].getModuleDriveOutput());
    SmartDashboard.putNumber("Left Back Drive Output", swerve_modules[Wheels.LEFT_BACK.ordinal()].getModuleDriveOutput());
    SmartDashboard.putNumber("Right Front Drive Output", swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getModuleDriveOutput());
    SmartDashboard.putNumber("Right Back Drive Output", swerve_modules[Wheels.RIGHT_BACK.ordinal()].getModuleDriveOutput());

    SmartDashboard.putNumber("tid", this.limelight.getAprilTagID());
  }
}
