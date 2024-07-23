// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCmd extends SequentialCommandGroup {

  /** Creates a new AutoShootCmd. */
  public AutoShootCmd(ShooterSubsystem shooter, IntakeSubsystem intake) {
    addCommands(
      new ShootCmd(shooter, intake));
  }
}
