// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final double kDeadband = 0.08; // Operator deadband
    public static final int kXboxDrivePort = 0; // Xbox port
    public static final int kXboxButtonPort = 1; // Xbox port
    public static final int kLeftJoystickPort = 2; // Xbox port
    public static final int kRightJoystickPort = 3; // Xbox port

  }

  public static class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // Module wheel diameter in meters
    public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;
    public static final double kDriveMotorGearRatio = 6.75; // Module drive motor gear ratio
    public static final double kSteerMotorGearRatio = 12.8; // Module steer motor gear ratio
    
    public static double kFeedforwardGainSteer = 0.11; // The feed forward gain for the module steer control

    public static Slot0Configs getSteerMotorGains() { 
      Slot0Configs kSteerMotorGains = new Slot0Configs();
      kSteerMotorGains.withKP(30); // The proportional gain for the module steer control
      return kSteerMotorGains;
    }
    
    public static Slot0Configs getDriveMotorGains() { 
      Slot0Configs kSteerMotorGains = new Slot0Configs();
      kSteerMotorGains.withKP(0.1); // The proportional gain for the module steer control TODO: Find
      return kSteerMotorGains;
    }

    public static SimpleMotorFeedforward leftFrontFF = new SimpleMotorFeedforward(0.21599, 2.2476, 0.040257);
    public static SimpleMotorFeedforward leftBackFF = new SimpleMotorFeedforward(0.20676, 2.1653, 0.16537);
    public static SimpleMotorFeedforward rightFrontFF = new SimpleMotorFeedforward(0.1788, 2.257, 0.036611);
    public static SimpleMotorFeedforward rightBackFF = new SimpleMotorFeedforward(0.11961, 2.3274, 0.13714);

    public static double kModuleAngleDeadband = 0.001;

    /* Swerve Current Limiting */
    public static final int kSteerCurrentLimit = 25;
    public static final int kSteerCurrentThreshold = 30;
    public static final double kSteerCurrentThresholdTime = 0.1;
    public static final boolean kSteerEnableCurrentLimit = true;

    public static final int kDriveCurrentLimit = 30;
    public static final int kDriveCurrentThreshold = 40;
    public static final double kDriveCurrentThresholdTime = 0.1;
    public static final boolean kDriveEnableCurrentLimit = true;
  }
  public static class ChassisConstants { 
    // Ports for driving motors
    public static final int kLeftFrontDriveID = 11; // CAN ID
    public static final int kRightFrontDriveID = 13; // CAN ID
    public static final int kLeftBackDriveID = 12; // CAN ID
    public static final int kRightBackDriveID = 10; // CAN ID
    // Ports for angle motors
    public static final int kLeftFrontSteerID = 21; // CAN ID
    public static final int kRightFrontSteerID = 23; // CAN ID
    public static final int kLeftBackSteerID = 22; // CAN ID
    public static final int kRightBackSteerID = 20; // CAN ID
    // Ports for encoders 
    public static final int kLeftFrontEncID = 31; // CAN ID
    public static final int kRightFrontEncID = 33; // CAN ID
    public static final int kLeftBackEncID = 32; // CAN ID
    public static final int kRightBackEncID = 30; // CAN ID
    // Offsets for absolute encoders in rotations (i.e: 360 degrees = 1 rotation):
    public static final double kLeftFrontOffset = -0.284912109375; 
    public static final double kRightFrontOffset = -0.599609375;
    public static final double kLeftBackOffset = -0.4755859375;
    public static final double kRightBackOffset = -0.18603515625;
    // Which motors are inverted:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   public static final boolean frontLeftDriveInverted = true;
    public static final boolean kLeftFrontInverted = false; 
    public static final boolean kRightFrontInverted = false;
    public static final boolean kLeftBackInverted = false;
    public static final boolean kRightBackInverted = false;

    public static final double kMaxDrivingVelocity = 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSec = 5;
    public static final double kTeleDriveMaxSpeedMetersPerSec = 4;
    public static final double kTeleDriveMaxAngulerSpeedRadiansPerSec = Math.PI*1.5;

    // Distance between centers of right and left wheels on robot meters
    public static final double kTrackWidth = 0.5403;
    // Distance between front and back wheels on robot meters
    public static final double kWheelBase = 0.5403;
    // Distance between middle of robot to module wheel
    public static final double kWheelRadius = 0.38205;

    // Swerve Kinematics:
    public static final SwerveDriveKinematics kDriveKinematics =
      new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), //Left front
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //Right front
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //Left back
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2) //Right back
      ); 
  }

  public static class ShooterConstants {
    public static final int kShooterMotorPort = 50;

    public static final int kPistonForwardChannelNumber = 1;
    public static final int kPistonReverseChannelNumber = 2;

    public static final double kShootPower = -0.51;//Desired speed for the movement of the wheel in firing
    public static final double kCollectPower = 0.3;//Desired speed for the movement of the wheel in collection
  }
  public static class IntakeConstants{
    // Intake motor ports:
    public static final int kIntakeMotorPort = 40;
    public static final int kAngleMotorPort = 41;

    // Intake analog ports:
    public static final int kPieceDetectorPort = 0;

    // Intake digital ports:
    public static final int kAngleEncoderPort = 0;
    public static final int kOpenLimitSwitchPort = 1;
    public static final int kClosedLimitSwitchPort = 2;

    // Limit constants:
    public static final double kPieceDetectorDetectionThreshold = 1.45;
    public static final double kAngleEncoderOffset = 0.75;
    
    // Intake motor speeds:
    public static final double kMotorOutputPower = -0.7;
    public static final double kAmpOutputPower = -0.88;
    public static final double kMotorIntakePower = 0.5;

    // Angle motor positions:
    public static final double kOpenRotations = 0.0973;
    public static final double kAmpRotations = 0.361;
    public static final double kClosedRotations = 0.597;

    // Angle controller constants:
    public static final double kP = 2; 
    public static final double kMaxVelocityRotPerSec = 0.65;
    public static final double kMaxAccelerationRotPerSecSquared = 1;
    public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(
      IntakeConstants.kMaxVelocityRotPerSec,
      IntakeConstants.kMaxAccelerationRotPerSecSquared);
  }
}