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
  public static class ClimberSubsystemConstants{
    public static final int motor1Port=10;
    public static final int motor2Port=11;
    public static final int limSwitchRightPort=0;
    public static final int limSwitchLeftPort=1;
    public static final double maxMotorPower=0.7;
    public static final double kpForLift TODO
    public static final double ksForLift TODO
    public static final double motorPowerPrep TODO
    public static final int ticsForRotation TODO
    public static final double lengthForRotation =125.6637061435917;
  } 
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  }

}
