// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShootCmd;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootMoveCmd extends SequentialCommandGroup {

  /** Creates a new AutoShootMoveCmd. */
  public AutoShootMoveCmd(ShooterSubsystem shooter, IntakeSubsystem intake, ChassisSubsystem chassis) {
    // Add commands to the command group in sequential order
    addCommands(
      /** 1. Execute the shooting command to shoot the game piece */
      new ShootCmd(shooter, intake),

      /** 2. Drive the chassis forward at a speed of 1 for 2 seconds */
      new InstantCommand(() -> chassis.drive(1, 0, 0, false)), // Move forward at full speed
      new WaitCommand(2), // Wait for 2 seconds to allow the robot to drive forward

      /** 3. Stop the chassis after the wait */
      new InstantCommand(() -> chassis.drive(0, 0, 0, false)) // Stop all movement
    );
  }
}
