// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Util.LEDController;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AimShootCmd extends SequentialCommandGroup {

  /** Creates a new AimShootCmd. */
  public AimShootCmd(ChassisSubsystem chassis, IntakeSubsystem intake, ShooterSubsystem shooter, Supplier<Boolean> backButton) {
    addCommands(
      new InstantCommand(() -> LEDController.setActionState(LEDController.ActionStates.SPEAKER_AIMING)),
      new AimAtAprilTagCmd(chassis, Robot.allianceColor == "BLUE"?7:4, backButton),
      new ShootCmd(shooter, intake));
  }
}
