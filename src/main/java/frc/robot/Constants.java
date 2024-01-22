// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
  public static class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 6.75;
    public static final double kSteerMotorGearRatio = 12.8;

    public static Slot0Configs getSteerMotorGains() {
      Slot0Configs kSteerMotorGains = new Slot0Configs();
      kSteerMotorGains.withKP(0); // TODO: Find kP
      kSteerMotorGains.withKS(0); // TODO: Find power needed to overcome static friction
      return kSteerMotorGains;
    }
  }
  public static class ChassisConstants { 
    // Ports for driving motors TODO: Find
    public static final int kLeftFrontDriveID = 11; // CAN ID
    public static final int kRightFrontDriveID = 13; // CAN ID
    public static final int kLeftBackDriveID = 12; // CAN ID
    public static final int kRightBackDriveID = 10; // CAN ID
    // Ports for angle motors TODO: Find
    public static final int kLeftFrontSteerID = 21; // CAN ID
    public static final int kRightFrontSteerID = 23; // CAN ID
    public static final int kLeftBackSteerID = 22; // CAN ID
    public static final int kRightBackSteerID = 20; // CAN ID
    // Ports for encoders TODO: Find
    public static final int kLeftFrontEncID = 31; // CAN ID
    public static final int kRightFrontEncID = 33; // CAN ID
    public static final int kLeftBackEncID = 32; // CAN ID
    public static final int kRightBackEncID = 30; // CAN ID
    // Offsets for absolute encoders: TODO: Find
    public static final int kLeftFrontOffset = 0; 
    public static final int kRightFrontOffset = 0;
    public static final int kLeftBackOffset = 0;
    public static final int kRightBackOffset = 0;
    // Which motors are inverted TODO: Find                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     public static final boolean frontLeftDriveInverted = true;
    public static final boolean kLeftFrontInverted = false; 
    public static final boolean kRightFrontInverted = false;
    public static final boolean kLeftBackInverted = false;
    public static final boolean kRightBackInverted = false;

    public static final double kMaxDrivingVelocity = 1;

    // Distance between centers of right and left wheels on robot cm
    public static final double kTrackWidth = 54.03;
    // Distance between front and back wheels on robot cm
    public static final double kWheelBase = 54.03;

    // Swerve Kinematics TODO: Choose positions with + -
    public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                    new Translation2d(kWheelBase / 2, kTrackWidth / 2), //Left front
                    new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //Right front
                    new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //Left back
                    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); //Right back
  }
}
