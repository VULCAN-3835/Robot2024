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

public class AutoShootCollectForwardShotCmd extends SequentialCommandGroup {

  /** Creates a new AutoShootCollectForwardShotCmd. */
  public AutoShootCollectForwardShotCmd(ShooterSubsystem shooter, IntakeSubsystem intake, ChassisSubsystem chassis) {
    // Declare the chassis subsystem as a requirement for this command group.
    addRequirements(chassis);
    
    addCommands(
      /** 1. Execute the auto shoot and collect command  */
      new AutoShootCollectForwardCmd(shooter,intake,chassis),

      /** 2. Drive the chassis backward at a speed of -1 for 1.85 seconds */
      new InstantCommand(() -> chassis.drive(-2, 0, 0, false)),

      /** 3. wait until the robot gets to the shooting position */
      new WaitCommand(1.75),

      /** 4. Stop the chassis after the backward drive */
      new InstantCommand(() -> chassis.drive(0, 0, 0, false)),

      /** 5. Wait for 1 second before shooting again */
      new WaitCommand(0.5),

      /** 6. Execute the shooting command again */
      new ShootCmd(shooter, intake),

      /** 7. Wait for 1 second after shooting */
      new WaitCommand(1),

      /** 8. Drive the chassis forward at a speed of 1 for 3 seconds */
      new InstantCommand(() -> chassis.drive(1, 0, 0, false)),
      new WaitCommand(3),

      /** 9. Stop the chassis after the forward drive */
      new InstantCommand(() -> chassis.drive(0, 0, 0, false))
    );
  }
}
