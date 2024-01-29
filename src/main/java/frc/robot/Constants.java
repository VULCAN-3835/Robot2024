// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ClimberConstants{
    public static final int kLeftMotorPort = 40;
    public static final int kRightMotorPort = 41;
    public static final int kLeftSwitchPort = 0;
    public static final int kRightSwitchPort = 1;

    public static final double kMaxMotorPower=0.7;
    public static final double kMaxElevatorHeight = 45.524; // CM

    public static final int kTicksPerRotation =2048;
    public static final double kLengthForRotation = 12.56637061435917;// Diameter on Cm
    public static final double kMotorRatio = 28; // this is for multipling

    // TODO: Find values
    public static final double kElevatorMaxCruiseVelocity = 0;
    public static final double kElevatorAcceleration = 0;

    // TODO: Find motor direction
    public static final boolean kLeftInverted = false;
    public static final boolean kRightInverted = false;

    public static Slot0Configs getElevatorSlot() {
      Slot0Configs configs = new Slot0Configs();
      configs.withKS(0); // TODO: Find voltage to overcome static friction
      configs.withKP(0.7); // TODO: Find real proportion

      return configs;
    }
}

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  }

}

