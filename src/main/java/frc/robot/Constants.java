// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class TrapConstants {
      public final static int kTrapShooterMotorPort = 1;//TODO:change port to actual port
      public final static int kLimitSwitchPort = 1;//TODO: change port to actual port
      public final static int kPieceLimitSwitch =1;//TODO: change port to actual port
      public final static int kDoubleSelenoidForward = 0;//TODO: change port to actual port
      public final static int kDoubleSelenoidReverse = 1;//TODO: change port to actual port
      public final static double kMotorSpeed = 0;//TODO: change speed to actual speed
      public final static double kMotorSpeedCollector = -1;//TODO: change speed to actual speed
  }
} 
