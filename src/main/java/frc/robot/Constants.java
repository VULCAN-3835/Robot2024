// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.security.PublicKey;

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
    public static final int kDriverControllerPort = 0;
  }

  public static class ShooterConstants {
      public static final int KDoubleSelenoidForward=1;
      public static final int KDoubleSelenoidReverse=1;
      public static final int KMotorShooterPort=0;
      public static final int KticksPerRotation =2048 ;
      public static final double kkPshooter=0.5;
      public static final double kSDspeedRPM=100;//Desired speed for the movement of the wheel in firing
      public static final double kCDspeedRPM=-100;//Desired speed for the movement of the wheel in collection
      public static double kDspeed=0.0;//The speed the shooting is currently aiming to be
  /** Creates a new ShooterSubsystem. */
  }

  
}
